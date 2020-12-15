tx = 0:0.05:15;
ty = 0:0.02:6;
y = e.^(-ty);
x = cos(1*2*pi*tx);
z = y.*x;
plot(tx, z);

state=0;

periode=[];

for i = 1:1:length(tx)
  if state==0
    if z(i)<0
      periode(end+1)=i;
      state=1;
    endif
  endif
  if state==1
    if z(i)>0
      periode(end+1)=i;
      state=0;
    endif
    
  endif

endfor


disp(periode)