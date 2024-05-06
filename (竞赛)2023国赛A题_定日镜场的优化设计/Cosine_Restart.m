function T = Cosine_Restart(step,warmup_step,number_of_restarts,restart_mult,restart_dec,max_T,min_T)
%步数，热机次数，重启次数，   


    T = zeros(1,step);                  %初始化
    l = 1;
    r = warmup_step;      
    
    if warmup_step > 0
        T(l:r) = linspace(min_T,max_T,warmup_step);
    end
    
    left_step = step - warmup_step;     %除去热机步数剩下步数
    if restart_mult == 1                
        %如果重启次数1次
        peri = floor(left_step / (number_of_restarts + 1));
    else 
        %如果多次重启
        peri = floor(left_step * (1 - restart_mult) / (1 - restart_mult^(number_of_restarts + 1)));
    end
    for i = 0:number_of_restarts        %一个一个周期地生成
        if i == number_of_restarts
            %最后一个周期的长度要特判
            len = step - r;
        else
            len = peri * restart_mult ^ i;
        end
        dec = restart_dec ^ i;          %确定当前周期的衰减程度
        l = r + 1;          
        r = l + len - 1;                %确定当前周期的左右端点
        tmp_T = zeros(1,r-l+1);     
        for j = 0:len-1
            %函数生成
            tmp_T(j+1) = dec * (min_T + 0.5*(max_T-min_T)*(1+cos(j*pi/(len-1))));
        end
        T(l:r) = tmp_T(1:len);          %合并
    end
end