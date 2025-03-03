function [W00 , W0, W1, W2, W3, W4] = FCN3F(W00 , W0, W1, W2, W3, W4, X, D)
alpha = 0.01; %학습률

N = 300;% 데이터 수
for k = 1:N
    x = X(k, :)';       % (8x1)
    d = D(k, :)';       % (784x1)
    % 1st Hidden Layer
    v1 = W00*x;          % (300x8)*(8X1)->(300X1)
    y1 = Sigmoid(v1);   % (300X1),sigmoid 활성화 함수
    % 2st Hidden Layer
    v2 = W0*y1;         % (400x300)*(300X1)->(400X1)
    y2 = Sigmoid(v2);   % (400X1),sigmoid 활성화 함수
    % 3rd Hidden Layer
    v3 = W1*y2;         % (500x400)*(400X1)->(500X1)
    y3 = Sigmoid(v3);   % (500X1),sigmoid 활성화 함수
    % Output Layer
    v4 = W2*y3;          % (600x500)*(500X1)->(600X1)
    y4= Sigmoid(v4);      % (600X1),sigmoid 활성화 함수

    v5 = W3*y4;          % (700x600)*(600X1)->(700X1)
    y5= Sigmoid(v5); 

    v = W4*y5;          % (784x700)*(700X1)->(784X1)
    y= Sigmoid(v);      % (784X1)

     
    % Output Layer
    e = d - y;
    delta = e;
    
    % 3rd Hidden Layer
    e3 = W4'*delta;          % (700x1)
    delta3 = y5.*(1-y5).*e3; % (700x1)
    
    % 2nt Hidden Layer
    e2 = W3'*delta3;         % (600x1)
    delta2 = y4.*(1-y4).*e2; % (600x1)

    % 1st Hidden Layer
    e1 = W2'*delta2;         % (500x1)
    delta1 = y3.*(1-y3).*e1; % (500x1))

    e0 = W1'*delta1;          % (400x1)
    delta0 = y2.*(1-y2).*e0; %  (400x1)

    e00 = W0'*delta0;          % (300x1)
    delta00 = y1.*(1-y1).*e00; % (300x1)
    

   
    dW00 = alpha*delta00*x';   % (1X1)*(300X1)*(1X8)->(300X8)
    W00 = W00 + dW00;           % W1 가중치 업데이트

    dW0 = alpha*delta0*y1';  % (1X1)*(400X1)*(1X300)->(400X300)
    W0 = W0 + dW0;           % W2 가중치 업데이트

    dW1 = alpha*delta1*y2';  % (1X1)*(500X1)*(1X400)->(500X400)
    W1 = W1 + dW1;           % W3 가중치 업데이트

    dW2 = alpha*delta2*y3';   % (1X1)*(600X1)*(1X500)->(600X500)
    W2 = W2 + dW2;           % W4 가중치 업데이트

    dW3 = alpha*delta3*y4';   % (1X1)*(700X1)*(1X600)->(700X600)
    W3 = W3 + dW3;           % W4 가중

    dW4 = alpha*delta*y5';   % (1X1)*(784X1)*(1X700)->(784X700)
    W4 = W4 + dW4;           % W4 가중
end
end