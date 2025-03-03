clear all
% 10000개의 숫자 영상 불러오기
Images = loadMNISTImages('C:\Users\82102\Desktop\기말\인공지능 교수\t10k-images.idx3-ubyte');
Images = reshape(Images, 28, 28, []);   % 10000개의 28x28 영상 구조로 변경
% 10000개의 숫자 영상에 대한 정답 0 ~ 9 라벨 불러오기
Labels = loadMNISTLabels('C:\Users\82102\Desktop\기말\인공지능 교수\t10k-labels.idx1-ubyte');
% 라벨값 0-> 10 변환
Labels(Labels == 0) = 10;    % 0 --> 10



% Learning
%
W1 = 1e-2*randn([9 9 20]);  % Convolution filter weight matrix
W5 = (2*rand(100, 2000) - 1) * sqrt(6) / sqrt(100 + 2000);
Wo = (2*rand( 10,  100) - 1) * sqrt(6) / sqrt( 10 +  100);

%학습시킬 8000개의 숫자 영상 불러오기
X = Images(:, :, 1:8000);
%학습시킬 8000개의 숫자 영상에 대한 라벨값 불러오기
D = Labels(1:8000);

for epoch = 1:50
  epoch
  [W1, W5, Wo, d] = CNN1F(W1, W5, Wo, X, D);
end


save('CNN1 result_two.mat');

load('CNN1 result_two.mat');

% Test

%테스트할 2000개의 숫자 영상 불러오기
X = Images(:, :, 8001:10000);
%테스트할 2000개의 숫자 영상에 대한 라벨값 불러오기
D = Labels(8001:10000);

acc = 0;
N   = length(D);
for k = 1:N

  x = X(:, :, k);                   % Input,                28x28
  y1 = Conv(x, W1);                 %                       20x20x20
  y2 = ReLU(y1);                    % ReLU,                 20x20x20
  y3 = MaxPool(y2);                 % MAXPooling,           10x10x20
  y4 = reshape(y3, [], 1);          %                       2000  
  v5 = W5*y4;                       % (100X2000)*(2000X1) = 100X1
  y5 = ReLU(v5);                    %  ReLU,                100X1
  v  = Wo*y5;                       % (10x100)*(100X1)=     10X1 
  y  = Softmax(v);                  % Softmax,              10X1

  [~, i] = max(y);                  % y의 최대값의 인덱스를 반환

  %정확도 계산
  if i == D(k)
    acc = acc + 1;
  end
end

%정확도 출력
acc = acc / N;
fprintf('정확도: %.2f%%\n', acc*100);


