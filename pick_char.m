function c=pick_char(char_weights)
% Select characteristic point based off of weight of point
val=rand();
for i=1:length(char_weights)
    if val<sum(char_weights(1:i))
        c=i;
        break
    end
end



end