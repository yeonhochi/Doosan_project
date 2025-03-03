function [W1, W2, W3, W4] = FCN1F(W1, W2, W3, W4, X, D)
alpha = 0.1; %학습률

N = 10;% 데이터 수
for k = 1:N
    x = X(k, :)';       % (10x1)
    d = D(k, :)';       % (4x1)
    % 1st Hidden Layer
    v1 = W1*x;          % (20x10)*(10X1)->(20X1)
    y1 = Sigmoid(v1);   % (20X1),sigmoid 활성화 함수
    % 2st Hidden Layer
    v2 = W2*y1;         % (30x20)*(20X1)->(30X1)
    y2 = Sigmoid(v2);   % (30X1),sigmoid 활성화 함수
    % 3rd Hidden Layer
    v3 = W3*y2;         % (40x30)*(30X1)->(40X1)
    y3 = Sigmoid(v3);   % (40X1),sigmoid 활성화 함수
    % Output Layer
    v = W4*y3;          % (4x40)*(40X1)->(4X1)
    y= Sigmoid(v);      % (4X1),sigmoid 활성화 함수

     
    % Output Layer
    e = d - y;
    delta = e;
    
    % 3rd Hidden Layer
    e3 = W4'*delta;          % (40x1)
    delta3 = y3.*(1-y3).*e3; % (40x1)
    
    % 2nt Hidden Layer
    e2 = W3'*delta3;         % (30x1)
    delta2 = y2.*(1-y2).*e2; % (30x1)

    % 1st Hidden Layer
    e1 = W2'*delta2;         % (20x1)
    delta1 = y1.*(1-y1).*e1; % (20x1)   
   
    dW1 = alpha*delta1*x';   % (20x10)
    W1 = W1 + dW1;           % W1 가중치 업데이트

    dW2 = alpha*delta2*y1';  % (30X20)
    W2 = W2 + dW2;           % W2 가중치 업데이트

    dW3 = alpha*delta3*y2';  % (40x30)
    W3 = W3 + dW3;           % W3 가중치 업데이트

    dW4 = alpha*delta*y3';   % (4x40)
    W4 = W4 + dW4;           % W4 가중치 업데이트
end
end