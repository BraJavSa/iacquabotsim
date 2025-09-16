#!/usr/bin/env python3
import rospy
import numpy as np
import casadi as ca
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose2D, Vector3, PoseStamped

# --------------------------------------------------------
# Modelo no lineal de propulsor (cmd -> fuerza)
# e inversa (fuerza -> cmd) con bisección
# --------------------------------------------------------
class ThrusterModel:
    def __init__(self):
        # Coeficientes identificados
        self.A_pos = 0.000001
        self.K_pos = 40.0209
        self.B_pos = 2.6249
        self.v_pos = 0.1615
        self.C_pos = 0.9432
        self.M_pos = 0.00001
        self.maxFwd = 36.3827

        self.A_neg = -31.4990
        self.K_neg = -0.00001
        self.B_neg = 3.6986
        self.v_neg = 0.3264
        self.C_neg = 0.9713
        self.M_neg = -1.0
        self.maxRev = -28.4393

    def force_from_cmd(self, cmd: float) -> float:
        if cmd > 0.01:
            T = self.A_pos + (self.K_pos - self.A_pos) / \
                (self.C_pos + np.exp(-self.B_pos * (cmd - self.M_pos)))**(1.0/self.v_pos)
        elif cmd < -0.01:
            T = self.A_neg + (self.K_neg - self.A_neg) / \
                (self.C_neg + np.exp(-self.B_neg * (cmd - self.M_neg)))**(1.0/self.v_neg)
        else:
            T = 0.0
        return max(min(T, self.maxFwd), self.maxRev)

    def cmd_from_force(self, T_des: float, tol=1e-2) -> float:
        lo, hi = -1.0, 1.0
        for _ in range(50):  # más iteraciones para precisión
            mid = 0.5*(lo+hi)
            T_mid = self.force_from_cmd(mid)
            if abs(T_mid - T_des) < tol:
                return mid
            if T_mid < T_des:
                lo = mid
            else:
                hi = mid
        return 0.5*(lo+hi)

# --------------------------------------------------------
# Nodo principal MPC
# --------------------------------------------------------
class DockingMPC:
    def __init__(self):
        rospy.init_node("mpc_docking")

        # -----------------------------
        # Parámetros MPC
        # -----------------------------
        self.N = 15         # horizonte
        self.dt = 0.2       # tiempo de muestreo

        # Parámetros identificados (ζ_i)
        self.zeta = {
            1: 124.59, 2: 120.03, 3: 141.52,
            4: 25.93,  5: 35.98,  6: 37.02,
            7: 25.79,  8: 200.29, 9: 18.53
        }

        # Límites de actuadores
        self.Tu_max = 144.0   # surge máx con 4 thrusters
        self.Tv_max = 0.0     # sin thruster lateral
        self.Tr_max = 90.0    # momento de yaw aprox

        # Estado [x,y,psi,u,v,r]
        self.state = np.zeros(6)
        self.goal = np.array([0,0,0])

        # Modelo de propulsor
        self.thruster = ThrusterModel()

        # -----------------------------
        # Interfaces ROS
        # -----------------------------
        rospy.Subscriber("/iacquabot/sensors/position/p3d_wamv",
                         Odometry, self.odom_cb)
        rospy.Subscriber("/docking_goal", Pose2D, self.goal_cb)
        self.pub_cmd = rospy.Publisher("/boat/cmd_thruster",
                                       Vector3, queue_size=10)
        self.pub_path = rospy.Publisher("/mpc_pred_path",
                                        Path, queue_size=1)

        # Construcción del solver MPC
        self._build_mpc()

    # -----------------------------
    # Callbacks
    # -----------------------------
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0-2.0*(q.y*q.y + q.z*q.z)
        psi = np.arctan2(siny, cosy)

        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        r = msg.twist.twist.angular.z

        self.state = np.array([x, y, psi, u, v, r])

    def goal_cb(self, msg):
        self.goal = np.array([msg.x, msg.y, msg.theta])

    # -----------------------------
    # Modelo dinámico Fossen
    # -----------------------------
    def dynamics(self, x, u):
        z = self.zeta
        x_pos,y_pos,psi,u_b,v_b,r_b = x[0],x[1],x[2],x[3],x[4],x[5]
        Tu,Tv,Tr = u[0],u[1],u[2]

        M = ca.diag(ca.vertcat(z[1],z[2],z[3]))
        C = ca.vertcat(
            ca.horzcat(0,0,-(z[2])*v_b),
            ca.horzcat(0,0,(z[1])*u_b),
            ca.horzcat((z[2])*v_b,-(z[1])*u_b,0)
        )
        D = ca.diag(ca.vertcat(
            -(z[4]+z[5]*ca.fabs(u_b)),
            -(z[6]+z[7]*ca.fabs(v_b)),
            -(z[8]+z[9]*ca.fabs(r_b))
        ))

        nu = ca.vertcat(u_b,v_b,r_b)
        tau = ca.vertcat(Tu,Tv,Tr)
        nu_dot = ca.mtimes(ca.inv(M),(tau - ca.mtimes(C,nu) - ca.mtimes(D,nu)))

        J = ca.vertcat(
            ca.horzcat(ca.cos(psi),-ca.sin(psi),0),
            ca.horzcat(ca.sin(psi), ca.cos(psi),0),
            ca.horzcat(0,0,1)
        )
        eta_dot = ca.mtimes(J,nu)

        return ca.vertcat(eta_dot,nu_dot)

    def discretize(self, x,u):
        return x + self.dt*self.dynamics(x,u)

    # -----------------------------
    # Construcción del MPC
    # -----------------------------
    def _build_mpc(self):
        N = self.N
        nx = 6
        nu = 3

        X = ca.SX.sym("X",nx,N+1)
        U = ca.SX.sym("U",nu,N)
        X0 = ca.SX.sym("X0",nx)
        REF = ca.SX.sym("REF",3)

        cost = 0
        g = [X[:,0]-X0]

        w_pos,w_yaw,w_nu,w_tau = 10,5,0.1,0.01

        for k in range(N):
            xk = X[:,k]
            uk = U[:,k]
            x_next = X[:,k+1]
            g.append(x_next - self.discretize(xk,uk))

            pos_err = (xk[0]-REF[0])**2 + (xk[1]-REF[1])**2
            yaw_err = (xk[2]-REF[2])**2
            cost += w_pos*pos_err + w_yaw*yaw_err \
                    + w_nu*ca.sumsqr(xk[3:6]) + w_tau*ca.sumsqr(uk)

        g = ca.vertcat(*g)
        OPT_vars = ca.vertcat(ca.reshape(X,-1,1),ca.reshape(U,-1,1))

        lbx, ubx = [], []
        for _ in range((N+1)*nx):
            lbx.append(-ca.inf); ubx.append(ca.inf)
        for _ in range(N):
            lbx += [-self.Tu_max, -self.Tv_max, -self.Tr_max]
            ubx += [ self.Tu_max,  self.Tv_max,  self.Tr_max]

        nlp = {"x":OPT_vars,"f":cost,"g":g,"p":ca.vertcat(X0,REF)}
        opts = {"ipopt.print_level":0,"print_time":0}
        self.solver = ca.nlpsol("solver","ipopt",nlp,opts)
        self.lbx = np.array(lbx); self.ubx = np.array(ubx)
        self.nx=nx; self.nu=nu

    # -----------------------------
    # Paso del controlador
    # -----------------------------
    def step(self):
        x0 = self.state
        ref = self.goal
        p = np.concatenate([x0,ref])

        x_init = np.tile(x0,(self.N+1,1)).flatten()
        u_init = np.zeros((self.N,self.nu)).flatten()
        guess = np.concatenate([x_init,u_init])

        sol = self.solver(x0=guess,lbx=self.lbx,ubx=self.ubx,lbg=0,ubg=0,p=p)
        opt = sol["x"].full().flatten()
        start_u = (self.N+1)*self.nx
        u0 = opt[start_u:start_u+self.nu]

        # Fuerzas deseadas
        Tu, Tv, Tr = u0
        left_force  = Tu/2 - Tr/2
        right_force = Tu/2 + Tr/2

        # Conversión a comandos
        left_cmd  = self.thruster.cmd_from_force(left_force)
        right_cmd = self.thruster.cmd_from_force(right_force)

        msg = Vector3()
        msg.x = right_cmd
        msg.y = left_cmd
        msg.z = 0.0
        self.pub_cmd.publish(msg)

        # -----------------------------
        # Publicar trayectoria predicha
        # -----------------------------
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        X_opt = opt[:(self.N+1)*self.nx].reshape((self.nx,self.N+1))
        for k in range(self.N+1):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = X_opt[0,k]
            pose.pose.position.y = X_opt[1,k]
            pose.pose.position.z = 0.0
            path.poses.append(pose)
        self.pub_path.publish(path)

        rospy.loginfo_throttle(2.0,
            f"MPC -> F_left={left_force:.2f}N, F_right={right_force:.2f}N, " +
            f"cmd_left={left_cmd:.2f}, cmd_right={right_cmd:.2f}")

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

if __name__=="__main__":
    DockingMPC().run()
