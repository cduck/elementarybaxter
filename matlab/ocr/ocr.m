while 1
    try
        imread('in.png');
        run('ocr_pp');
        delete('in.png');
        fprintf('Restarting...\n')
        pause(1);
    catch err
        %fprintf(getReport(err));
        %fprintf('\n');
    end
end