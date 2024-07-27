function u = linear_mpc_controller(state, u_prev, params)
    %TODO: implement linear MPC
    V = params.V; 
    u_max = params.u_max;

    Ts= 0.1;
    N = 20;
    r = 1;

    % State-space
    x = [state(3); state(2)]; 

    A = [1 0; V*Ts 1];
    B = [Ts; 0.5*V*Ts^2];
    C = [0 1];
    
    %Predictive Model
    G = zeros(N,2);
    for i = 1:N
        if i==1
            G(i,:) = C*A;
        else
            G(i,:) = G(i-1,:) * A;
        end
    end

    H = zeros(N,N);
    for i = 2:N
        for j = 1:i-1
            H(i,j) = C * A^(i-j-1) * B;
        end
    end

    F = zeros(N,1);
    for i = 1:N
        F(i,:) = C*A^(i-1)*B;
    end
    
    D = eye(N:N);
    for i = i:N
        if i == N
            D(i,i) = 0;
        else
            D(i,i+1) = -1;
        end
    end

    %QP
    Q = r * (D' * D) + (H' * H);
    f = H' * (G * x + F * u_prev);

    A_const = [eye(N,N); -eye(N,N)];
    B_const = ones(2*N,1) * u_max;

    U_opt = quadprog(Q,f,A_const,B_const);
    u = U_opt(1);
end