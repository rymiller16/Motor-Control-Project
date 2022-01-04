function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('\ta: Read current sensor (ADC counts)\tb: Read current sensor (mA)\n\tc: Read encoder (counts)\t\td: Read encoder (deg)\n\te: Reset encoder\t\t\tf: Set PWM (-100 to 100)\n\tg: Set current gains\t\t\th: Get current gains\n\ti: Set position gains\t\t\tj: Get position gains\n\tk: Test current control\t\t\tl: Go to angle (deg)\n\tm: Load step trajectory\t\t\tn: Load cubic trajectory\n\to: Execute trajectory\t\t\tp: Unpower the motor\n\tq: Quit\t\t\t\t\tr: Get mode\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection  
        
        case 'a'        %%outputting ADC counts
            ADC_counts = fscanf(mySerial, '%d');
            fprintf('The number of ADC counts is %d: \n\n',ADC_counts);
        
        case 'b'
            current_ma = fscanf(mySerial, '%f');
            fprintf('The current (in mA) is %f: \n\n',current_ma);
            
        case 'c'        %%outputting motor angle in encoder counts
            counts = fscanf(mySerial, '%d');
            fprintf('The motor angle is %d counts: \n\n', counts);
            
        case 'd'        %%outputting motor angle in degrees
            encoder_degrees = fscanf(mySerial, '%f');
            fprintf('The motor angle is %f degrees: \n\n', encoder_degrees);   
                
        case 'e'        %%resetting encoder counts
            fprintf('The encoder has been reset. \n\n');
        
        case 'f'        %setting PWM duty cycle
            DC = input('Input integer PWM value (-100 to 100): ');
            fprintf(mySerial, '%d\n',DC);
            
            if DC ~= 0
                if (DC < 0)
                    if (DC < -100)
                        DC = -100;
                    end
                else
                    if (DC > 100)
                        DC = 100;
                    end 
                end 
            end
            
            fprintf('The duty cycle you have input is %d \n',DC);
        
        case 'g'        %%setting current gains
           i = input('Input the proportional gain constant for current control (Kp): ');
           j = input('Input the integral gain constant for current control (Ki): ');
             
           fprintf(mySerial, '%f\n', i);                    %send Kp,KI over UART
           fprintf(mySerial, '%f\n', j);
            
           Kp = fscanf(mySerial,'%f');                            %get Kp back over UART      
           Ki = fscanf(mySerial,'%f');                                              %get Ki back over UART
            
           fprintf('You have entered (for current gains) Kp: %.3f, Ki: %.3f \n\n',Kp,Ki);
            
        case 'h'        %%Get current gains 

           Kp = fscanf(mySerial,'%f');                             %get Kp back over UART
           Ki = fscanf(mySerial,'%f');                             %get Ki back over UART
            
           fprintf('Current current gains are Kp: %.3f, Ki: %.3f \n\n',Kp,Ki);
           
        case 'i'
            
           z = input('Input the proportional gain constant for position control (Kp): ');
           x = input('Input the integral gain constant for position control (Ki): ');
           y = input('Input the integral gain constant for position control (Kd): ');
             
           fprintf(mySerial, '%f\n', z);                    %send Kp,KI over UART
           fprintf(mySerial, '%f\n', x);
           fprintf(mySerial, '%f\n', y);
            
           Kp_pos = fscanf(mySerial,'%f');                            %get Kp back over UART      
           Ki_pos = fscanf(mySerial,'%f');                            %get Ki back over UART
           Kd_pos = fscanf(mySerial,'%f');
            
           fprintf('You have entered (for position gains) Kp: %.3f, Ki: %.3f, Kd: %.3f \n\n',Kp_pos,Ki_pos,Kd_pos);
           
        case 'j'
            
           Kp_pos = fscanf(mySerial,'%f');                             %get Kp back over UART
           Ki_pos = fscanf(mySerial,'%f');                             %get Ki back over UART
           Kd_pos = fscanf(mySerial,'%f');
            
           fprintf('Current position gains are Kp: %.3f, Ki: %.3f, Kd: %.3f \n\n',Kp_pos,Ki_pos,Kd_pos);
           
        case 'k'
            %note the function provided is gets number of samples first
            %from Serial, then gets two integers in successive form
            read_plot_matrix(mySerial);
            
        case 'l'
            v = input('Input desired angle (in degrees): ');
            fprintf(mySerial, '%f\n', v);
            
            Deg_ref = fscanf(mySerial,'%f'); 
            
            fprintf('The desired angle you have entered is: %.3f \n\n',Deg_ref);
            
        case 'm' %load step trajectory 
            A = input('Enter step trajectory of form [time, angle; time, angle]: (first time = 0 & last and first velocities = 0)');
            fprintf('Plotting step trajectory');
            step = genref(A,'step');
            Num_samps_step = length(step);
            fprintf(mySerial, '%d\n', Num_samps_step); 
            
            i = 0;
            while(i<Num_samps_step)
                fprintf(mySerial, '%f\n', step(i)); 
                i=i+1;
            end
            fprintf('Reference positions sent \n\n');
            
        case 'n' %load cubic trajectory
            B = input('Enter cubic trajectory of form [time, angle; time, angle]: (first time = 0 & last and first velocities = 0)');
            fprintf('Plotting cubic trajectory');
            cubic = genref(B,'cubic');
            Num_samps_cubic = length(cubic);
            fprintf(mySerial, '%d\n', Num_samps_cubic); 
            
            j = 0;
            while(j<Num_samps_cubic)
                fprintf(mySerial, '%f\n', cubic(i)); 
                j=j+1;
            end
            fprintf('Reference positions sent \n\n');
            
        case 'o' %execute trajectory
            read_plot_matrix_pos(mySerial);
            
        case 'r'        %%outputting current mode
            mode_str = fscanf(mySerial,'%s');
            fprintf('The current mode is %s: \n\n', mode_str);
        
        case 'p'
            fprintf('The motor has been turned off\n\n');
                        
        case 'q'        %%quitting
            has_quit = true;  
            
        otherwise       %%default selection: invalid
            fprintf('Invalid Selection %c\n\n', selection);
    end
end

end