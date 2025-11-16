function [E, A_P, B_P] = ImplicitLinmod(MY_FUN, XDOTo, Xo, Uo, DXDOT, DX, DU)
% IMPLICITLINMOD - Numerically linearizes an implicit nonlinear model.
%
%   [E, A, B, P, B_P] = ImplicitLinmod(MY_FUN, XDOT0, X0, U0, DXDOT, DX, DU)
%
%   MY_FUN  : function handle for your nonlinear model (e.g. @RCAM_model_implicit)
%   XDOT0   : nominal state derivative vector (n×1)
%   X0      : nominal state vector (n×1)
%   U0      : nominal control vector (m×1)
%   DXDOT   : perturbation matrix for XDOT (n×n)
%   DX      : perturbation matrix for X (n×n)
%   DU      : perturbation matrix for U (m×m)
%
%   Returns:
%     E, A, B  : standard linearized system matrices for implicit form
%     P, B_P   : auxiliary matrices sometimes used in further analysis
%
%   Based on implementation by Christopher Lum (University of Washington)
%   for the RCAM (Research Civil Aircraft Model) tutorial.

% -------------------------------------------------------------------------
% Determine system dimensions
% -------------------------------------------------------------------------
n = length(XDOTo);
m = length(Uo);

% -------------------------------------------------------------------------
% Initialize matrices
% -------------------------------------------------------------------------
E   = zeros(n, n);


% -------------------------------------------------------------------------
% -------------------- CALCULATE E MATRIX ---------------------------------
% -------------------------------------------------------------------------
for i = 1:n
    for j = 1:n
        % Obtain perturbation magnitude
        dxdot = DXDOT(i, j);

        % Define perturbation vectors
        xdot_plus  = XDOTo; 
        xdot_minus = XDOTo;

        xdot_plus(j)  = xdot_plus(j)  + dxdot;
        xdot_minus(j) = xdot_minus(j) - dxdot;

        % Evaluate function for perturbed xdot
        F = feval(MY_FUN, xdot_plus, Xo, Uo);
        F_plus_keep = F(i);

        F = feval(MY_FUN, xdot_minus, Xo, Uo);
        F_minus_keep = F(i);

        % Central difference approximation
        E(i, j) = (F_plus_keep - F_minus_keep) / (2 * dxdot);
    end
end

% -------------------------------------------------------------------------
% -------------------- CALCULATE A MATRIX ---------------------------------
% -------------------------------------------------------------------------
A_P = zeros(n, n);

for i = 1:n
    for j = 1:n
        dx = DX(i, j);

        x_plus  = Xo;
        x_minus = Xo;

        x_plus(j)  = x_plus(j)  + dx;
        x_minus(j) = x_minus(j) - dx;

        F = feval(MY_FUN, XDOTo, x_plus, Uo);
        F_plus_keep = F(i);

        F = feval(MY_FUN, XDOTo, x_minus, Uo);
        F_minus_keep = F(i);

        A_P(i, j) = (F_plus_keep - F_minus_keep) / (2 * dx);
    end
end

% -------------------------------------------------------------------------
% -------------------- CALCULATE B MATRIX ---------------------------------
% -------------------------------------------------------------------------
% Initialize the B_P matrix
B_P = zeros(n, m);

% Fill in each element of the matrix individually
for i = 1:n
    for j = 1:m
        % Obtain the magnitude of the perturbation to use
        du = DU(i, j);

        % Define perturbation vector. The current column determines
        % which element of u we are perturbing.
        u_plus  = Uo;
        u_minus = Uo;

        u_plus(j)  = u_plus(j)  + du;
        u_minus(j) = u_minus(j) - du;

        % Calculate F(row) for (xdoto, x0, u_plus)
        F = feval(MY_FUN, XDOTo, Xo, u_plus);
        F_plus_keep = F(i);

        % Calculate F(row) for (xdoto, x0, u_minus)
        F = feval(MY_FUN, XDOTo, Xo, u_minus);
        F_minus_keep = F(i);

        % Calculate Bprime(row, col)
        B_P(i, j) = (F_plus_keep - F_minus_keep) / (2 * du);
    end
end
