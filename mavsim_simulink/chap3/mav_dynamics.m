function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
%{
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];
%}
% initialize the initial conditions
%
x0  = [...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    0;...
    ];
%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pe    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = 13.5;
    n     = uu(6);
        
    %define the moments of inertia
    Jx   = 0.824;
    Jy   = 1.135;
    Jz   = 1.759;
    Jxz  = 0.120;
    
    %find gammas
    Gamma  = Jx*Jz-Jxz^2;
    Gamma1 = (Jxz*(Jx-Jy+Jz))/Gamma;
    Gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/Gamma;
    Gamma3 = Jz/Gamma;
    Gamma4 = Jxz/Gamma;
    Gamma5 = (Jz-Jx)/Jy;
    Gamma6 = Jxz/Jy;
    Gamma7 = (Jx*(Jx-Jy)+Jxz*Jxz)/Gamma;
    Gamma8 = Jx/Gamma;
    
    %find the vector
    pndot = (cos(theta)*cos(psi)*u) + ((sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v) + (cos(phi)*sin(theta)*cos(psi) +sin(phi)*sin(psi))*w;
    pedot = (cos(theta)*sin(psi)*u) + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v  + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*w;
    pddot = -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;
    udot = (r*v-q*w) + fx/m;
    vdot = (p*w-r*u) + fy/m;
    wdot = (q*u-p*v) + fz/m;
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    thetadot = cos(phi)*q - sin(phi)*r;
    psidot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    
    pdot = Gamma1*p*q - Gamma2*q*r + Gamma3*ell + Gamma4*n 
    qdot = Gamma5*p*r - Gamma6*(p^2-r^2) + m/Jy;
    rdot = Gamma7*p*q - Gamma1*q*r + Gamma4*ell + Gamma8*n
    
    %[pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot]

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];


% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%

function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
