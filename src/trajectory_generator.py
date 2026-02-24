import numpy as np
from scipy.interpolate import splprep, splev, BSpline
import matplotlib.pyplot as plt

class TrajectoryGenerator:
    """
    Generate smooth trajectory from waypoints using  
splines
    Complete implementation with velocity and acceleration profiles
    """
    
    def __init__(self, waypoints):
        self.waypoints = np.array(waypoints)
        self.trajectory_duration = None  # seconds
        self.max_velocity = None  # m/s
        self.max_acceleration = None  # m/s^2

    ##############################################################
    #### TODO - Implement spline trajectory generation ###########
    #### TODO - Ensure velocity and acceleration constraints #####
    #### TODO - Add member functions as needed ###################
    ##############################################################
    def generate_bspline_trajectory(self, num_points=None):
        """
        Generate spline trajectory with complete velocity/acceleration profiles
        
        Parameters:
        - num_points: number of points in the smooth trajectory
        
        Returns:
        - trajectory_points: numpy array of shape (num_points, 3)
        - time_points: numpy array of time stamps
        - velocities: numpy array of velocities (num_points, 3)
        - accelerations: numpy array of accelerations (num_points, 3)
        """
        print("Generating spline trajectory...")

        trajectory_points = None
        time_points = None
        velocities = None
        accelerations = None

        ############## IMPLEMENTATION STARTS HERE ##############
         # ----- Minimum-snap (order-7) piecewise polynomial with C^3 continuity -----
        wp = np.asarray(self.waypoints, dtype=float)
        assert wp.ndim == 2 and wp.shape[1] == 3 and len(wp) >= 2, "Need >=2 waypoints [N,3]"
        n = len(wp)                 # number of waypoints
        S = n - 1                   # number of segments
        order = 7                   # 7th order poly per segment
        coeffs_per_seg = order + 1  # 8 coefficients per segment
        dim = 3

        # ---------------- Timing allocation ----------------
        seg_len = np.linalg.norm(wp[1:] - wp[:-1], axis=1)
        # Avoid zero-length segments (duplicate waypoints) breaking time allocation
        seg_len[seg_len == 0.0] = 1e-9
        L = float(np.sum(seg_len))

        # Velocity/acc defaults if not provided
        v_cap = 2.0 if (self.max_velocity is None) else float(self.max_velocity)
        a_cap = 1.0 if (self.max_acceleration is None) else float(self.max_acceleration)

        # Lower bound on time via trapezoidal profile assuming path length L
        t_acc = v_cap / a_cap
        d_acc = 0.5 * a_cap * t_acc**2
        if L >= 2 * d_acc:
            T_min = 2 * t_acc + (L - 2 * d_acc) / v_cap
        else:
            t_peak = np.sqrt(L / a_cap)
            T_min = 2 * t_peak

        if self.trajectory_duration is None or float(self.trajectory_duration) < T_min:
            T_total = T_min
        else:
            T_total = float(self.trajectory_duration)

        # Distribute total time proportional to segment length
        Ti = T_total * (seg_len / np.sum(seg_len))
        Ti[Ti < 1e-4] = 1e-4  # avoid degenerate durations

        # --------------- Helpers (local to this method) ---------------
        def poly_der_coeffs(k):
            """Return derivative multipliers for kth derivative of t^i basis."""
            c = np.zeros(coeffs_per_seg)
            for i in range(k, coeffs_per_seg):
                val = 1.0
                for r in range(k):
                    val *= (i - r)
                c[i] = val
            return c

        def row_for(seg_idx, t_local, k):
            """
            Constraint row for derivative k at local time t for segment seg_idx.
            Uses correct basis: t^{i-k} for i>=k (zero otherwise).
            """
            row = np.zeros(S * coeffs_per_seg)
            base = seg_idx * coeffs_per_seg

            # multipliers for k-th derivative of t^i
            d = np.zeros(coeffs_per_seg)
            for i in range(k, coeffs_per_seg):
                val = 1.0
                for r in range(k):
                    val *= (i - r)
                d[i] = val

            # powers: t^{i-k}
            pow_k = np.zeros(coeffs_per_seg)
            if t_local == 0.0:
                # t^0 = 1 at i=k; higher powers vanish at 0
                if k < coeffs_per_seg:
                    pow_k[k] = 1.0
            else:
                for i in range(k, coeffs_per_seg):
                    pow_k[i] = t_local ** (i - k)

            row[base:base + coeffs_per_seg] = d * pow_k
            return row

        def Q_snap_block(T):
            """
            Build Q for ∫_0^T (p''''(t))^2 dt for one 7th-order poly.
            snap = Σ c_i a_i t^{i-4}, c_i = i(i-1)(i-2)(i-3), i>=4
            Q_ij = c_i c_j * T^{i+j-7}/(i+j-7)
            """
            Qb = np.zeros((coeffs_per_seg, coeffs_per_seg))
            c = np.zeros(coeffs_per_seg)
            for i in range(4, coeffs_per_seg):
                c[i] = i * (i - 1) * (i - 2) * (i - 3)
            for i in range(4, coeffs_per_seg):
                for j in range(4, coeffs_per_seg):
                    pwr = i + j - 7
                    Qb[i, j] = c[i] * c[j] * (T**pwr) / pwr
            return Qb

        # --------------- Assemble Q (block diagonal over segments) ---------------
        Q = np.zeros((S * coeffs_per_seg, S * coeffs_per_seg))
        for s in range(S):
            Qs = Q_snap_block(Ti[s])
            i0 = s * coeffs_per_seg
            Q[i0:i0 + coeffs_per_seg, i0:i0 + coeffs_per_seg] = Qs

        # tiny ridge to avoid numerical singularities
        Q += 1e-10 * np.eye(Q.shape[0])

        # --------------- Equality constraints Aeq x = beq ----------------
        rows = []
        rhs_tags = []  # ('pos', idx) -> equals waypoint idx value, ('zero', None) -> 0

        # (1) Position constraints at each segment start/end
        for s in range(S):
            rows.append(row_for(s, 0.0, k=0));         rhs_tags.append(('pos', s))
            rows.append(row_for(s, Ti[s], k=0));       rhs_tags.append(('pos', s + 1))

        # (2) C^3 continuity (v,a,j) at internal knots between seg s and s+1
        for s in range(S - 1):
            for k in (1, 2, 3):
                rows.append(row_for(s, Ti[s], k) - row_for(s + 1, 0.0, k))
                rhs_tags.append(('zero', None))

        # (3) Boundary derivatives: v=a=j=0 at global start and end (customize if needed)
        for k in (1, 2, 3):
            rows.append(row_for(0, 0.0, k));           rhs_tags.append(('zero', None))
        for k in (1, 2, 3):
            rows.append(row_for(S - 1, Ti[-1], k));    rhs_tags.append(('zero', None))

        Aeq = np.vstack(rows)
        M = Aeq.shape[0]

        # KKT solver for min 0.5 x^T Q x s.t. A x = b
        def solve_axis(b_axis):
            K = np.zeros((Q.shape[0] + M, Q.shape[1] + M))
            K[:Q.shape[0], :Q.shape[1]] = Q
            K[:Q.shape[0], Q.shape[1]:] = Aeq.T
            K[Q.shape[0]:, :Q.shape[1]] = Aeq
            rhs = np.zeros(Q.shape[0] + M)
            rhs[Q.shape[0]:] = b_axis
            try:
                sol = np.linalg.solve(K, rhs)
            except np.linalg.LinAlgError:
                # fallback for degeneracies
                sol, *_ = np.linalg.lstsq(K, rhs, rcond=None)
            return sol[:Q.shape[0]].reshape(S, coeffs_per_seg)

        # Build beq for each axis from tags
        b_all = np.zeros((M, dim))
        for ax in range(dim):
            vals = []
            for tag, idx in rhs_tags:
                if tag == 'pos':
                    vals.append(wp[idx, ax])
                else:
                    vals.append(0.0)
            b_all[:, ax] = np.array(vals)

        # Solve coefficients per axis
        coeffs = np.zeros((dim, S, coeffs_per_seg))
        for ax in range(dim):
            coeffs[ax] = solve_axis(b_all[:, ax])

        # --------------- Sampling --------------------------------------
        if num_points is None:
            N = int(max(200, np.ceil(T_total * 50)))  # ~50 Hz, at least 200
        else:
            N = int(num_points)

        # Samples per segment proportional to duration
        Ni = np.maximum(2, np.round(N * (Ti / np.sum(Ti))).astype(int))
        diff = int(np.sum(Ni) - N)
        if diff != 0:
            # adjust counts to hit exactly N
            idxs = np.argsort(Ti)[::-1] if diff > 0 else np.argsort(Ti)
            for k in range(abs(diff)):
                Ni[idxs[k % S]] -= np.sign(diff)

        pts, vels, accs, t_global = [], [], [], []
        t0 = 0.0

        for s in range(S):
            t_loc = np.linspace(0.0, Ti[s], int(Ni[s]), endpoint=(s == S - 1))
            for tl in t_loc:
                # position basis
                powers = np.array([tl**i for i in range(coeffs_per_seg)])

                # p'(t) = sum_{i>=1} i * a_i * t^{i-1}
                pow_d1 = np.zeros(coeffs_per_seg)
                if tl == 0.0:
                    pow_d1[1] = 1.0
                else:
                    for i in range(1, coeffs_per_seg):
                        pow_d1[i] = tl ** (i - 1)
                d1_mult = np.array([0] + [i for i in range(1, coeffs_per_seg)])
                dp = d1_mult * pow_d1

                # p''(t) = sum_{i>=2} i*(i-1) * a_i * t^{i-2}
                pow_d2 = np.zeros(coeffs_per_seg)
                if tl == 0.0:
                    pow_d2[2] = 1.0
                else:
                    for i in range(2, coeffs_per_seg):
                        pow_d2[i] = tl ** (i - 2)
                d2_mult = np.array([0, 0] + [i * (i - 1) for i in range(2, coeffs_per_seg)])
                ddp = d2_mult * pow_d2

                p = np.array([np.dot(coeffs[ax, s], powers) for ax in range(dim)])
                v = np.array([np.dot(coeffs[ax, s], dp)     for ax in range(dim)])
                a = np.array([np.dot(coeffs[ax, s], ddp)    for ax in range(dim)])
                pts.append(p); vels.append(v); accs.append(a); t_global.append(t0 + tl)
            t0 += Ti[s]

        trajectory_points = np.vstack(pts)
        velocities = np.vstack(vels)
        accelerations = np.vstack(accs)
        time_points = np.array(t_global)

        # --------------- Final uniform time scaling for limits ----------
        vmag = np.linalg.norm(velocities, axis=1)
        amag = np.linalg.norm(accelerations, axis=1)
        scale = 1.0
        if np.any(vmag > v_cap + 1e-9):
            scale = max(scale, float(np.max(vmag) / v_cap))
        if np.any(amag > a_cap + 1e-9):
            scale = max(scale, float(np.sqrt(np.max(amag) / a_cap)))
        if scale > 1.0:
            time_points *= scale
            velocities /= scale
            accelerations /= (scale**2)
            Ti *= scale
            T_total *= scale

        # Cache for visualization text/lines
        self.trajectory_duration = float(T_total)
        self.max_velocity = v_cap
        self.max_acceleration = a_cap

        return trajectory_points, time_points, velocities, accelerations
            

 
    def visualize_trajectory(self, trajectory_points=None, velocities=None, 
                           accelerations=None, ax=None):
        """Visualize the trajectory with velocity and acceleration vectors"""
        if ax is None:
            fig = plt.figure(figsize=(15, 5))
            ax1 = fig.add_subplot(131, projection='3d')
            ax2 = fig.add_subplot(132)
            ax3 = fig.add_subplot(133)
            standalone = True
        else:
            ax1 = ax
            standalone = False
        
        if trajectory_points is not None:
            # Plot 3D trajectory
            ax1.plot(trajectory_points[:, 0], trajectory_points[:, 1], 
                    trajectory_points[:, 2], 'b-', linewidth=2, label='Spline Trajectory')
            
            # Plot waypoints
            ax1.plot(self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2], 
                    'ro-', markersize=8, linewidth=2, label='Waypoints')
            
            # Plot velocity vectors (sampled)
            if velocities is not None:
                step = max(1, len(trajectory_points) // 20)  # Show ~20 vectors
                for i in range(0, len(trajectory_points), step):
                    pos = trajectory_points[i]
                    vel = velocities[i] * 0.5  # Scale for visualization
                    ax1.quiver(pos[0], pos[1], pos[2], 
                             vel[0], vel[1], vel[2], 
                             color='green', alpha=0.7, arrow_length_ratio=0.1)
            
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_zlabel('Z (m)')
            ax1.set_title('3D Trajectory')
            ax1.legend()
        
        if standalone and velocities is not None and accelerations is not None:
            # Plot velocity magnitude over time
            time_points = np.linspace(0, self.trajectory_duration, len(velocities))
            vel_magnitudes = np.linalg.norm(velocities, axis=1)
            ax2.plot(time_points, vel_magnitudes, 'g-', linewidth=2)
            ax2.axhline(y=self.max_velocity, color='r', linestyle='--', 
                       label=f'Max Vel: {self.max_velocity} m/s')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Velocity (m/s)')
            ax2.set_title('Velocity Profile')
            ax2.grid(True)
            ax2.legend()
            
            # Plot acceleration magnitude over time
            acc_magnitudes = np.linalg.norm(accelerations, axis=1)
            ax3.plot(time_points, acc_magnitudes, 'm-', linewidth=2)
            ax3.axhline(y=self.max_acceleration, color='r', linestyle='--', 
                       label=f'Max Acc: {self.max_acceleration} m/s²')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Acceleration (m/s²)')
            ax3.set_title('Acceleration Profile')
            ax3.grid(True)
            ax3.legend()
            
            plt.tight_layout()
            plt.show()
        
        return ax1 if not standalone else None