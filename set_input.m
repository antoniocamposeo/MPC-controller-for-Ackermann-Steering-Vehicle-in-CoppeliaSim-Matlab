function set_input(sim,clientID,mv)
 val = sim.simxPackFloats(round(mv',3).*[1 1]); 
 [returnCode] = sim.simxSetStringSignal(clientID,'signal',val,sim.simx_opmode_blocking);
 if (returnCode==sim.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        %fprintf('tutto ok\n');
    else
        %fprintf('non tutto ok\n');
    end
end