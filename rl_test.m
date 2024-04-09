classdef rl_test < rl.env.MATLABEnvironment
    %MYENVIRONMENT: Template for defining custom environment in MATLAB.    
    
    %% Properties (set properties' attributes accordingly)
    properties
        % Specify and initialize environment's necessary properties    
        % Acceleration due to gravity in m/s^2 in water
        Gravity = 4.9;
        
        % Mass oprf the jellyfish
        PearlMass = 0.02;
        
        % DutyCycle and Frequency limits
        MaxDC = 0.8
        MinDC = 0.2
        MaxF = 1
        MinF = 0.5
               
        % Sample time
        Ts = 0.1
        
        % Errors to fail the episode (m)
        ErrorThreshold = 0.01
        stability=20;
        goal_Depth
        offset=0.21
        
        % Rewards
        RewardForReaching = 3
        RewardForZ=0.3
        RewardForZdot=0.4
        RewardForStability=0.9
        RewardForNoAction=0.7
        
        % Penalties
        PenaltyForX = -0.5
        PenaltyForY = -0.5
        PenaltyForAction=-0.5

        %
        SerialPort
        intrins
        cam
        filelogger
        logdir
        ep_num
        name
    end
    
    properties
        % Initialize system state [z,zdot]'
        State = zeros(2,1)
          %check the serial port and che
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false       
        IsProgress = false
        prevPos=0
        current_Error=0;
        prev_Error=0;
        prev_DC=0;
        timer=0;
        count=0;
        prev_count=0;
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = rl_test()

            
            % this.filelogger.
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([2 1]);
            ObservationInfo.Name = 'Pearl States';
            ObservationInfo.Description = 'z, zdot';
            
            
            % Initialize Action settings   
            ActionInfo = rlFiniteSetSpec([0 0.2 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.75 0.8]);
            ActionInfo.Name = 'Duty Cycle Action';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            
            this.count=0;

            this.filelogger=rlDataLogger();
            setup(this.filelogger);
            this.logdir=fullfile(pwd, "data/rltraining/");


            
            this.filelogger.LoggingOptions.LoggingDirectory = this.logdir;
            this.filelogger.LoggingOptions.FileNameRule = "episode<id>";
            this.filelogger.AgentStepFinishedFcn=@step;

            % Initialize property values and pre-compute necessary values
            updateActionInfo(this);
            this.SerialPort=serialport("COM6", 115200);
            cameraParams=load("calib_data_second\right_cam_clib.mat");
            this.intrins=cameraParams.cameraParamsRight.Intrinsics;
            this.cam=webcam("HD Web Camera");
            
            tet_im=snapshot(this.cam);
            imshow(tet_im);
            close all;
            this.timer=0;
            this.ep_num=1;
            
            
                        
        end

        function InitialObservation = reset(this)
            disp("Pausing for sinking 12 secs");
            writeline(this.SerialPort, "0;0");
            info=zeros(3,3);
            while info(1, 3)<=0.87
                info=getRobInfo(this);
                if info(1, 3)>=0.87
                    break;
                end
            end
            disp("Resetting")
            this.goal_Depth=whichdepth();
            info=getRobInfo(this);
            InitialObservation = [info(1,3); info(3, 3)];
            this.State = InitialObservation;
            disp("Got current Initial position")
            this.count=0;
            this.ep_num=this.ep_num+1;
            this.name="data/rltraining_"+num2str(this.ep_num)+"/";
            mkdir(this.name);
            this.prev_count=0;
            % (Optional) Use notifyEnvUpdated to signal that the 
            % environment is updated (for example, to update the visualization)
            notifyEnvUpdated(this);
        end

        function send_output(this, dc_out)
            % s=serialport("COM6", 115200);
            %should output DC value to the arduino
            disp("Freq 0.75 is maintained");
            disp(["UTR: ", dc_out]);
            output=string(dc_out)+";0.75";
            writeline(this.SerialPort, output);
            disp("pausing for the data to be sent");
            pause(1); %pause   
        end
     
        % Apply system dynamics and simulates the environment with the 
        % ISSUE HERE CUZ IDK THE SYSTEM DYNAMICS
        function [Observation,Reward,done,LoggedSignals] = step(this, Action)
            LoggedSignals = [];
            this.timer=this.timer+1;
            disp("Sending output")
            send_output(this, Action);  
            disp("getting the position");
            info=getRobInfo(this);
                % Log the action taken
            %got the current state 
            % [x y z; 
            % dx dy dz; 
            % xdot ydot zdot]
            
            Observation = [info(1,3); info(3, 3)];
            
            % Update system states
            this.State = Observation;
            disp("Updated states")
            % Check terminal condition
            Z = Observation(1); %take the current position
            current_error=Z-this.goal_Depth;
            % prev_Error=abs(abs(this.prevPos)-this.goal_Depth);
            disp("Calculated Errors")
            disp(current_error)
            temp = abs(current_error) < this.ErrorThreshold; %if current steady state error is less than the defined error threshold say its don
            disp(temp)
            if temp
                this.count=this.count+1;
                disp(this.count)
            end
            if this.count==this.stability
                this.IsDone=true;
                done=true;
                disp(this.IsDone);
            else
                this.IsDone=false; done=false;
            end
            % Get reward
            disp("Setting off to find the reward")
            Reward = getReward(this, info, current_error, Action);
            LoggedSignals=[info; Reward Action current_error; this.goal_Depth 0 0];
            filename=this.name+"Log_"+num2str(this.timer)+'.mat';
            save(filename,'LoggedSignals');
        end
        
        function info=getRobInfo(this)
            disp("In Rob info")
            %should capture snapshot and read the position of the robot 
            % capture two positions, get velocity and then the current
            % data matrix:
            % 1  2  3  4
            % x1 y1 z1 t1
            % x2 y2 z2 t2
           
            data=zeros(2, 4);
            info=zeros(3, 3);
            tic; j=1;
            while j<3
                disp("Starting while lop")
                img=snapshot(this.cam);
                % imshow(img);
                I = undistortImage(img,this.intrins, OutputView="same"); %undistorting
                I=rgb2gray(I); %converting to grey
                [id,loc,pose] = readAprilTag(I, "tag36h11", this.intrins, 12);
                % imshow(I);
        
                if(isempty(id))
                    disp("No Tag Detected");
                    continue
                end
            
                for i = 1:length(pose)
                    disp("in for loop");
                   
                    zpos= (pose(i).Translation(3))/1000; %obtaining depth
                    disp(zpos-0.21);
                   data(j,3)=zpos-this.offset;
                   data(j,2) = (pose(i).Translation(2))/1000;
                   data(j,1) = (pose(i).Translation(1))/1000;
                   data(j,4)=toc;
             
                end %pose loop end
                 j=j+1;
            end %ending the while loop

            %smoothing
            data(:,1)=smoothdata(data(:,1), "movmean");
            data(:,2)=smoothdata(data(:,2), "movmean");
            data(:,3)=smoothdata(data(:,3), "movmean");
            
            % x y z latest pos
            info(1, 1)=data(2, 1);
            info(1, 2)=data(2, 2);
            info(1, 3)=data(2, 3);

            %calculating the dels second row
            info(2,1)=diff(data(:,1));
            info(2,2)=diff(data(:,2));
            info(2,3)=diff(data(:,3));
            del_t=diff(data(:,4));
            
            %calculating the final row
            info(3, 1)=info(2,1)/del_t;
            info(3, 2)=info(2,2)/del_t;
            info(3, 3)=info(2,3)/del_t;      
            % output matrix: info
            % 1     2    3  
            % x     y    z 
            % dx   dy   dz 
            % xdot ydot  zdot

        end %ending the function
         
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        
       % Helper methods to create the environment
       % update the action info based on max force
        function updateActionInfo(this)
            this.ActionInfo.Elements = [0 0.2 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.75 0.8];
        end
        
        % Reward function
        function Reward = getReward(this, info, curr_error, Action)
            %a1, a2, a3, a4, a5
            % [x y z; we only need the last two rows here btw
            % dx dy dz; 
            % xdot ydot zdot]
            Reward=0;

            %rewarding z displacement
            % Reward=Reward+this.RewardForZ*info(2, 3); %remove

            %Rewarding Z velocity
            Reward=Reward+this.RewardForZdot*info(3, 3);

            %Rewarding no Action
            if(Action==0)
                disp(["Rewarding no action"]);
                Reward=Reward+this.RewardForNoAction;
                this.prev_DC=Action;
            end

            %rewarding stability
            if((this.prev_count-this.count)~=0)
                Reward=Reward+this.RewardForStability*(this.prev_count-this.count);
                this.prev_count=this.count;
            end

            %penalising the error using the coeffs for penalising x and y
            %motion
            del_error=abs(abs(this.prev_Error) - abs(curr_error));
            Reward= Reward + this.PenaltyForX*del_error + ...
                             this.PenaltyForY*abs(curr_error);
            this.prev_Error=curr_error;
            
            %Reward for Reaching
            if this.IsDone==true
                Reward = Reward + this.RewardForReaching;
            end
            disp(["Reward: ",Reward])
        end
        
        % (optional) Visualization method
        function plot(this)
            % Initiate the visualization
            
            % Update the visualization
            envUpdatedCallback(this)
        end
        
        % (optional) Properties validation through set methods
        function set.State(this,state)
            validateattributes(state,{'numeric'},{'finite','real','vector','numel',2},'','State');
            this.State = double(state(:));
            notifyEnvUpdated(this);
        end
        function set.Gravity(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Gravity');
            this.Gravity = val;
        end
        function set.PearlMass(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','CartMass');
            this.PearlMass = val;
        end
        function set.MaxDC(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','MaxDC');
            this.MaxDC = val;
            updateActionInfo(this);
        end
        function set.MinDC(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','MinDC');
            this.MinDC = val;
            updateActionInfo(this);
         end
        function set.MaxF(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','MaxFreq');
            this.MaxF = val;
            updateActionInfo(this);
          end
        function set.MinF(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','MinFreq');
            this.MinF = val;
            updateActionInfo(this);
        end
        function set.Ts(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Ts');
            this.Ts = val;
        end
        function set.ErrorThreshold(this,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','AngleThreshold');
            this.ErrorThreshold = val;
        end
        function set.RewardForReaching(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForReaching');
            this.RewardForReaching = val;
        end
        function set.RewardForZ(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForZ');
            this.RewardForZ = val;
        end
        function set.RewardForStability(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForStability');
            this.RewardForStability = val;
        end
        function set.RewardForZdot(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForZdot');
            this.RewardForZdot = val;
        end
        function set.RewardForNoAction(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForNoAction');
            this.RewardForNoAction = val;
        end
        function set.PenaltyForX(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForX');
            this.PenaltyForX = val;
        end
        function set.PenaltyForY(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForY');
            this.PenaltyForY = val;
        end
        function set.PenaltyForAction(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForAction');
            this.PenaltyForAction = val;
        end
        function set.goal_Depth(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','GoalDepth');
            this.goal_Depth = val;
        end
        function set.stability(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','Stability');
            this.stability = val;
         end
        function set.offset(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','Offset');
            this.offset = val;
         end
        function set.count(this,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','Counter');
            this.count = val;
        end
    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(this)
        end
    end
end

function depth=whichdepth()
    randomin=randi([1, 4]);
    switch randomin
        case 1
            depth=0.75;
        case 2
            depth=0.8;
        case 3
            depth=0.7;
        case 4
            depth=0.85;
    end
    disp(["goaldepth: ", depth]);
end