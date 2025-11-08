% Eval DWA traj
function score = mc_DWA(traj, goal, static_obs, robot_radius, dyn_obs, global_path, step_idx, dt, dynr, a_brake, robot_state)

    w_h = 1; w_d = 1; w_v = 1;          % weights

    end_pos = traj(end,1:2);           % final pos
    end_th  = traj(end,3);             % final theta
    end_v   = traj(end,4);             % final v

% ==== static obs check ====
    N = size(traj,1);
    for i = 1:N
        p = traj(i,1:2);
        for j = 1:size(static_obs,1)
            o = static_obs(j,:);
            x1 = o(1)-0.5-robot_radius; x2 = o(1)+0.5+robot_radius;
            y1 = o(2)-0.5-robot_radius; y2 = o(2)+0.5+robot_radius;
            if p(1)>=x1 && p(1)<=x2 && p(2)>=y1 && p(2)<=y2
                score = -1e10; return;
            end
        end
    end

% ==== global path consistency ====
    goal_dir = atan2(goal(2)-end_pos(2), goal(1)-end_pos(1));
    dh_g = abs(angdiff(end_th, goal_dir));
    sc_g = (pi-dh_g)/pi;

    final_dir = atan2(goal(2)-global_path(end-1,2), goal(1)-global_path(end-1,1));
    dh_f = abs(angdiff(end_th, final_dir));
    sc_f = (pi-dh_f)/pi;

    heading_sc = 0.7*sc_g + 0.3*sc_f;

    [~, idx] = min(vecnorm(global_path - robot_state(1:2),2,2));
    M = 5; idx_e = min(idx+M-1,size(global_path,1));
    loc_path = global_path(idx:idx_e,:);
    d_min = min(pdist2(traj(:,1:2),loc_path),[],2);
    path_sc = -mean(d_min);

    gp_sc = heading_sc + path_sc;

% ==== smoothness & stability ====
    dx = diff(traj(:,1)); dy = diff(traj(:,2));
    ddx = diff(dx); ddy = diff(dy);
    curv = abs(ddy./(dx(1:end-1).^2 + dy(1:end-1).^2 + 1e-3));
    curv_sc = -mean(curv);

    v_seq = traj(:,4);
    if length(v_seq)>=3
        acc_v = diff(v_seq)/dt; jerk_v = diff(acc_v)/dt; jv_sc = -mean(abs(jerk_v));
    else, jv_sc = 0; end

    w_seq = traj(:,5);
    if length(w_seq)>=3
        acc_w = diff(w_seq)/dt; jerk_w = diff(acc_w)/dt; jw_sc = -mean(abs(jerk_w));
    else, jw_sc = 0; end

    smooth_sc = curv_sc + jv_sc + jw_sc;

% ==== goal attraction ====
    alpha = 0.5;
    dist_sc = -exp(alpha*norm(goal-end_pos));

% ==== total score ====
    score = gp_sc + smooth_sc + dist_sc;
end