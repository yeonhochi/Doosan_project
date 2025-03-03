% clear all
% 
% %입력데이터 +,-,x 각각 10개씩
% for k = 1:30 
%     input1(:,:,k)=im2gray(im2double(imread(['C:\Users\82102\Desktop\인공지능 연산자\',num2str(k),'.png'])));
% end
% 
% % 라벨값 설정 ********더하기 -> 1, 빼기 -> 2, 곱하기-> 3
% Labels1 = repelem(1:3, 10);  % 1~10번째 이미지(더하기)는 라벨값 1, 11~20번째 이미지(빼기)는 라벨값 2, 21~30번째 이미지(곱하기)는 라벨값 3
% 
% 
% % Learning
% W1 = 1e-2 * randn([9 9 20]);  % Convolution filter weight matrix
% W2 = (2 * rand(100, 2000) - 1) * sqrt(6) / sqrt(100 + 2000);
% W3 = (2 * rand(500, 100) -1)* sqrt(6) / sqrt(500+ 100); 
% W4 = (2 * rand(1000, 500) -1)* sqrt(6) / sqrt(1000+ 500); 
% Wo = (2 * rand(3, 1000) - 1) * sqrt(6) / sqrt(3 + 1000);
% 
% X = input1(:, :, 1:30);
% D = Labels1(1:30);
% 
% for epoch = 1:1000
%     epoch
%     [W1, W2, W3, W4, Wo] = CNN3F(W1, W2, W3, W4, Wo, X, D);
% end
% 
% save('CNN3 result.mat');

load('CNN3 result.mat');


for k = 1:3 
    input2(:,:,k)=im2gray(im2double(imread(['C:\Users\82102\Desktop\인공지능 연산자 테스트\',num2str(k-1),'.png'])));
end
Labels2 = repelem(1:3, 1);

X = input2(:,:, 1:3);
D = Labels2(1:3);
acc = 0;
N   = length(D);


% 테스트 이미지에 대한 예측을 수행합니다
for k = 1:N
x=X(:,:,k);                    % Input,              28x28
y1 = Conv(x, W1);              %                     20x20x20
y2 = ReLU(y1);                 % ReLU,               20x20x20
y3 = MaxPool(y2);              % MAXPooling,         10x10x20
y4 = reshape(y3, [], 1);       %                     2000  
v5 = W2 * y4;                  % (100X2000)*(2000X1) = 100X1
y5 = ReLU(v5);                 %  ReLU,                100X1
v6 = W3 * y5;                  % (500X100)*(100X1) =   500X1
y6 = ReLU(v6);                 %  ReLU,                500X1
v7 = W4 * y6;                  % (1000X500)*(500X1) =  1000X1
y7 = ReLU(v7);                 %  ReLU,                1000X1

v = Wo * y7;                   % (3x1000)*(1000X1)=    3X1 
y = Softmax(v);                %                       3X1

[~, i] = max(y);               % y의 최대값의 인덱스를 반환
    if i == D(k)
        acc = acc + 1;
    end
end
% 정확도를 계산하고 출력합니다
acc = acc / N;
fprintf('정확도: %.2f %%\n', acc*100);



