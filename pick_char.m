function c=pick_char(char_weights)

val=rand();

for i=1:char_weights
    if val<sum(char_weights(1:i))
        c=i;
        break
    end
end



end