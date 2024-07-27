function state_dot = car_dynamics(t, state, params)
    V = params.V;
    u = params.u;
    
    theta = state(3); % state = [x,y,theta]
        
    state_dot = [V * 1;
                 V * theta;
                 u];
end