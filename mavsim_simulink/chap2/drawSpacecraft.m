function drawSpacecraft(uu,V,F,patchcolors)

     V = [...
    %fusalage and nosecone
    2    1    0;... % point 1
    2   -1    0;... % point 2
    2    1   -2;... % point 3
    2   -1   -2;... % point 4
                    -6   0   -1;... % point 5
    3    0   -1;...%point 6
    %wing
    1   -3   -1;...%point 7
                    -1   -3   -1;...%point 8
                    -1    3   -1;...%point 9
    1    3   -1;...%point 10
    %tail wing
                    -5   -1   -1;...%point 11
                    -6   -1   -1;...%point 12
                    -6    1    -1;...%point 13
                    -5    1    -1;...%point 14
    %tailfin
                    -6   0  -1;...%point 15
                    -5   0  -1;...%point 16
                    -6   0  -3;...%point 17
                ]';
            
  F = [...
    %fusalage
    1, 2,  3,  4;...  % front
    3, 4, 5, 3;...  % top
    2, 4, 5, 2;...%left
    1, 3, 5, 1;...%right
    1, 2, 5, 1;...%bottom
    %nosecone
    3, 4, 6, 3;... %top
    2, 4, 6, 2;...%left
    1, 3, 6, 1;...%right
    1, 2, 6, 1;...%bottom
    %wing
    7, 8, 9, 10;...
    %tailwing
    11, 12, 13, 14;...
    %tailfin
    15, 16, 17, 15;...
                        ];
            
                    
    patchcolors = [...
    %fusalage
                    myyellow;... % front
                    myblue;...   % back
                    myblue;...   % right
                    myblue;...   % left
                    myblue;...   % top
    %nosecone
                    myblue;... %top
                    myblue;...
                    myblue;...
                    myblue;...
    %wing
                    mygreen;...  
    %tailwing
                    myyellow;...
    %tailfin
                    myred;...
                    ];

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent spacecraft_handle;
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        spacecraft_handle = drawSpacecraftBody(V,F,patchcolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Spacecraft Test')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-1000,1000,-1000,1000,-1000,1000]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawSpacecraftBody(V,F,patchcolors,...
                           pn,pe,pd,phi,theta,psi,...
                           spacecraft_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
                                 
   V = [...
    %fusalage and nosecone
    2    1    0;... % point 1
    2   -1    0;... % point 2
    2    1   -2;... % point 3
    2   -1   -2;... % point 4
                    -6   0   -1;... % point 5
    3    0   -1;...%point 6
    %wing
    1   -3   -1;...%point 7
                    -1   -3   -1;...%point 8
                    -1    3   -1;...%point 9
    1    3   -1;...%point 10
    %tail wing
                    -5   -1   -1;...%point 11
                    -6   -1   -1;...%point 12
                    -6    1    -1;...%point 13
                    -5    1    -1;...%point 14
    %tailfin
                    -6   0  -1;...%point 15
                    -5   0  -1;...%point 16
                    -6   0  -3;...%point 17
                ]';
            
  F = [...
    %fusalage
    1, 2,  3,  4;...  % front
    3, 4, 5, 3;...  % top
    2, 4, 5, 2;...%left
    1, 3, 5, 1;...%right
    1, 2, 5, 1;...%bottom
    %nosecone
    3, 4, 6, 3;... %top
    2, 4, 6, 2;...%left
    1, 3, 6, 1;...%right
    1, 2, 6, 1;...%bottom
    %wing
    7, 8, 9, 10;...
    %tailwing
    11, 12, 13, 14;...
    %tailfin
    15, 16, 17, 15;...
                        ];
            
                    
    patchcolors = [...
    %fusalage
                    myyellow;... % front
                    myblue;...   % back
                    myblue;...   % right
                    myblue;...   % left
                    myblue;...   % top
    %nosecone
                    myblue;... %top
                    myblue;...
                    myblue;...
                    myblue;...
    %wing
                    mygreen;...  
    %tailwing
                    myyellow;...
    %tailfin
                    myred;...
                    ];
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
 if isempty(handle),
      handle = patch('Vertices', V, 'Faces', F,...
    'FaceVertexCData',patchcolors,...
    'FaceColor','flat',...
    'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi);
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

  