function so3mat = VecToso3(omg)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
so3mat = [   0    -omg(3)  omg(2);
          omg(3)     0    -omg(1);
         -omg(2)  omg(1)     0   ];
end
