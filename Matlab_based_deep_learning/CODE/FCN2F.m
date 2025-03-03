function [W1, W2, W3, W4] = FCN2F(W1, W2, W3, W4, X, D)
alpha = 0.1; %학습률

N = 300;% 데이터 수
for k = 1:N
    x = X(k, :)';       % (11x1)
    d = D(k, :)';       % (8x1)
    % 1st Hidden Layer
    v1 = W1*x;          % (50x11)*(11X1)->(50X1)
    y1 = Sigmoid(v1);   % (50X1),sigmoid 활성화 함수
    % 2st Hidden Layer
    v2 = W2*y1;         % (90x50)*(50X1)->(90X1)
    y2 = Sigmoid(v2);   % (90X1),sigmoid 활성화 함수
    % 3rd Hidden Layer
    v3 = W3*y2;         % (150x90)*(90X1)->(150X1)
    y3 = Sigmoid(v3);   % (150X1),sigmoid 활성화 함수
    % Output Layer
    v = W4*y3;          % (8x150)*(150X1)->(8X1)
    y= Sigmoid(v);      % (8X1),sigmoid 활성화 함수

     
    % Output Layer
    e = d - y;
    delta = e;
    
    % 3rd Hidden Layer
    e3 = W4'*delta;          % (150x1)
    delta3 = y3.*(1-y3).*e3; % (150x1)
    
    % 2nt Hidden Layer
    e2 = W3'*delta3;         % (90X1)
    delta2 = y2.*(1-y2).*e2; % (90X1)

    % 1st Hidden Layer
    e1 = W2'*delta2;         % (50X1)
    delta1 = y1.*(1-y1).*e1; % (50X1)
    

   
    dW1 = alpha*delta1*x';   % (50X11)
    W1 = W1 + dW1;           % W1 가중치 업데이트

    dW2 = alpha*delta2*y1';  % (90x50)
    W2 = W2 + dW2;           % W2 가중치 업데이트

    dW3 = alpha*delta3*y2';  % (150x90)
    W3 = W3 + dW3;           % W3 가중치 업데이트

    dW4 = alpha*delta*y3';   % (8x150)
    W4 = W4 + dW4;           % W4 가중치 업데이트
end
end