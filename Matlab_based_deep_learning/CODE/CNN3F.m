function [W1, W2, W3, W4, Wo] = CNN3F(W1, W2, W3, W4, Wo, X, D)
%
%

alpha = 0.01; %
beta  = 0.95; %

%MOMWNTUM사용하기
momentum1 = zeros(size(W1));
momentum2 = zeros(size(W2));
momentum3 = zeros(size(W3));
momentum4 = zeros(size(W4));
momentumo = zeros(size(Wo));

N = length(D);

bsize = 5; 
blist = 1:bsize:(N-bsize+1); 

% One epoch loop
%
for batch = 1:length(blist)
  dW1 = zeros(size(W1));
  dW2 = zeros(size(W2));
  dW3 = zeros(size(W3));
  dW4 = zeros(size(W4));
  dWo = zeros(size(Wo));
  
  % Mini-batch loop
  %
  begin = blist(batch);
  for k = begin:begin+bsize-1
    
    x = X(:,:, k ); % 28x28
    y1 = Conv(x, W1); %Convolution, 20x20x20
    y2 = ReLU(y1); %ReLu, 20x20x20
    y3 = MaxPool(y2); % 2x2 Mean pooling, 10x10x20 
    y4 = reshape(y3, [] ,1); % 10x10x20 -> 2000x1
    v5 = W2*y4; % (360x2000) x (2000x1) -> (360x1)
    y5 = ReLU(v5);% ReLU, 360x1
    v6 = W3*y5; % (800x360)x(360x1) -> (800x1) 
    y6 = ReLU(v6);% ReLU, 800x1
    v7 = W4*y6; % (1000x800)x(800x1) -> (1000x1) 
    y7 = ReLU(v7); % ReLU, 1000x1
    v = Wo*y7; % (3x1000) x (1000x1) -> (3x1)
    y = Softmax(v); % Softmax, (3x1)

    % One-hot encoding
    %
    d = zeros(3, 1);
    d(sub2ind(size(d), D(k), 1)) = 1;



    e = d - y; %output layer, 3x1
    delta = e; %3x1

    e7 = Wo'*delta; %Hidden layer 3, (1000x3)x(3x1) -> 1000x1
    delta7 = (y7 > 0 ) .* e7; %1000x1

    e6 = W4'*delta7; %Hidden layer 2, (800x1000)x(1000x1) -> 800x1
    delta6 = (y6 > 0) .* e6; %800x1

    e5 = W3'*delta6; %Hidden layer 1, (360x800)x(800x1) -> 360x1
    delta5 = (y5 > 0) .* e5; %360x1

    e4 = W2'*delta5; %Mean Pooling layer, (2000x360)x(360x1) -> 2000x1

    e3 = reshape(e4, size(y3)); %2000x1 -> 10x10x20

    e2 = zeros(size(y2)); %20x20x20
    W0 = ones(size(y2)) / (2*2); %1/4 (20x20x20)
    
    for c = 1:20
      e2(:, :, c) = kron(e3(:, :, c), ones([2 2])) .* W0(:, :, c);
    end
    
    delta2 = (y2 > 0) .* e2;          % ReLU layer
  
    delta1_x = zeros(size(W1));       % Convolutional layer
    for c = 1:20
      delta1_x(:, :, c) = conv2(x(:, :), rot90(delta2(:, :, c), 2), 'valid');
    end
    
    dW1 = dW1 + delta1_x; 
    dW2 = dW2 + delta5*y4';
    dW3 = dW3 + delta6*y5';
    dW4 = dW4 + delta7*y6';
    dWo = dWo + delta*y7';
  end 
  


  % Update weights
  %
  dW1 = dW1 / bsize;
  dW2 = dW2 / bsize;
  dW3 = dW3 / bsize;
  dW4 = dW4 / bsize;
  dWo = dWo / bsize;
  
  momentum1 = alpha*dW1 + beta*momentum1;
  W1        = W1 + momentum1;

  momentum2 = alpha*dW2 + beta*momentum2;
  W2        = W2 + momentum2;

  momentum3 = alpha*dW3 + beta*momentum3;
  W3        = W3 + momentum3;
  
  momentum4 = alpha*dW4 + beta*momentum4;
  W4        = W4 + momentum4;
   
  momentumo = alpha*dWo + beta*momentumo;
  Wo        = Wo + momentumo;  
end

end

