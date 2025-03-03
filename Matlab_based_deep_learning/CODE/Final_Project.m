clear all

%------------------------------CNN1-------------------------------------%.

%CNN1 (max pooling)
load('CNN1 result.mat');

%CNN1 출력 데이터 
D_CNN1=[];

for k = 0:9
  % 직접 그린 0~9까지 숫자 이미지 입력 input data, 28x28
  x = im2double(rgb2gray(imread(['C:\Users\82102\Desktop\테스트 숫자 이미지\',num2str(k),'.png']))); 
  y1 = Conv(x, W1);                 %                       20x20x20
  y2 = ReLU(y1);                    %                       20x20x20
  y3 = MaxPool(y2);                 % maxPooling,           10x10x20
  y4 = reshape(y3, [], 1);          %                       2000X1  
  v5 = W5*y4;                       % (100X2000)*(2000X1) = 100X1               
  y5 = ReLU(v5);                    % ReLU,                 100X1
  v  = Wo*y5;                       % (10x100)*(100X1)=     10X1 
  y  = Softmax(v);                  % Softmax,              10X1
  [~, index] = max(y);              % y의 최대값의 인덱스를 반환


  %onehot encoding 
  onehot_encoding = zeros(1, 10);   % 영행렬 선언
  onehot_encoding(index) = 1;       % 위에서 얻은 인덱스 값의 위치에 1할당 
  D_CNN1 = [D_CNN1; onehot_encoding];                   % 10x10
end

%결과 출력
D_CNN1


%------------------------------CNN2-------------------------------------%

%CNN2 (mean pooling)
load('CNN2 result.mat');
D_CNN2=[];
for k = 0:9
  % 직접 그린 0~9까지 숫자 이미지 입력 input data, 28x28
  x = im2double(rgb2gray(imread(['C:\Users\82102\Desktop\테스트 숫자 이미지\',num2str(k),'.png']))); 
  y1 = Conv(x, W1);                 %                    20x20x20
  y2 = ReLU(y1);                    % ReLU               20x20x20
  y3 = Pool(y2);                    % meanPooling,       10x10x20
  y4 = reshape(y3, [], 1);          % 2000  
  v5 = W5*y4;                       % (100X2000)*(2000X1) = 100X1
  y5 = ReLU(v5);                    % ReLU,                 100X1
  v  = Wo*y5;                       % (10x100)*(100X1)=     10X1
  y  = Softmax(v);                  % Softmax,              10X1
  [~, index] = max(y);              % y의 최대값의 인덱스를 반환

  onehot_encoding = zeros(1, 10);   % 영행렬 선언
  onehot_encoding(index) = 1;       % 위에서 얻은 인덱스 값의 위치에 1할당
  D_CNN2 = [D_CNN2; onehot_encoding];                      %10X10
end
D_CNN2

%------------------------------CNN3-------------------------------------%

%CNN3 (max pooling)
load('CNN3 result.mat');
D_CNN3=[];
for k = 1:3
% 직접 그린 +, -, X  연산자 이미지 각각 1개씩 입력 input data, 28x28
x=im2double(rgb2gray(imread(['C:\Users\82102\Desktop\인공지능 연산자 테스트\',num2str(k-1),'.png'])));

y1 = Conv(x, W1);              %                      20x20x20
y2 = ReLU(y1);                 % ReLU                 20x20x20
y3 = MaxPool(y2);              % maxPooling,          10x10x20
y4 = reshape(y3, [], 1);       % 2000 
v5 = W2 * y4;                  %(100X2000)*(2000X1) = 100X1
y5 = ReLU(v5);                 %                      100X1
v6 = W3 * y5;                  %(500X100)*(100X1) =   500X1
y6 = ReLU(v6);                 %                      500X1
v7 = W4 * y6;                  % (1000X500)*(500X1) = 1000X1
y7 = ReLU(v7);                 %                      1000X1 
v = Wo * y7;                   %  (3X1000)*(1000X1)=  3X1
y = Softmax(v);                %                      3X1

[~, index] = max(y);           % y의 최대값의 인덱스를 반환
 onehot_encoding = zeros(1, 3);% 영행렬 선언
 onehot_encoding(index) = 1;   % 위에서 얻은 인덱스 값의 위치에 1할당
 D_CNN3 = [D_CNN3; onehot_encoding]; % 3X3
end
D_CNN3


%------------------------------FCN1-----------------------------------------%

%FCN1,(CNN1의 출력을 입력으로 사용)
load('FCN1 result.mat');

X1=D_CNN1; %CNN_1의 출력
D_FCN1=[];
for k = 1:10
x = X1(k, :)';      % (10x1)

v1 = W1*x;          % (20x10)*(10X1)->(20X1)
y1 = Sigmoid(v1);   % (20X1)

v2 = W2*y1;         % (30x20)*(20X1)->(30X1)
y2 = Sigmoid(v2);   % (30X1)

v3 = W3*y2;         % (40x30)*(30X1)->(40X1)
y3 = Sigmoid(v3);   % (40X1)

v = W4*y3;          % (4x40)*(40X1)->(4X1)
y = round(Sigmoid(v));  % (4X1)

D_FCN1 = [D_FCN1; reshape(y, 4, [])']; %(10x4)
end
D_FCN1


%-----------------------------FCN2 입력데이터 생성(11bit)------------------------------------%

%  D_CNN3->CNN3출력, D_FCN1->FCN1출력
result = zeros(300, 11);
index = 1;
for i = 1:3
    for j = 1:10
        for k = 1:10
            % 3*3 행렬 한 행
            CNN3_3bit = D_CNN3(i, :);
            
            % 10*4 행렬 두 개 각각 한 행
            CNN1_4bit = D_FCN1(k, :);
            CNN2_4bit = D_FCN1(j, :);
                      
            % 행렬 합치기
            result(index, :) = [CNN3_3bit, CNN1_4bit, CNN2_4bit];
            % 연산자(3bit)+
            
            % 행 인덱스 증가
            index = index + 1;
        end
    end
end
X_FCN2=result;
X_FCN2



%------------------------------FCN2-------------------------------------%

%FCN2
load('FCN2 result.mat')
X4=X_FCN2;

D_FCN2=[];

N = 300;             % 데이터 수 
for k = 1:N
x = X4(k, :)';       % (4x1)

v1 = W1*x;          % (5x4)*(4X1)->(5X1)
y1 = Sigmoid(v1);   % (5X1)

v2 = W2*y1;         % (6x5)*(5X1)->(6X1)
y2 = Sigmoid(v2);   % (6X1)

v3 = W3*y2;         % (7x6)*(6X1)->(7X1)
y3 = Sigmoid(v3);   % (7X1)

v = W4*y3;          % (7x7)*(7X1)->(7X1)
y = round(Sigmoid(v));  % (7X1)

D_FCN2 = [D_FCN2; reshape(y, 8, [])'];

end
D_FCN2


%-------------------------------FCN3--------------------------------------%

X_FCN3=D_FCN2; %FCN2의 출력을 입력으로 사용

load('FCN3 result.mat')

N1 = 300;
for k1=1:N1
    x1 = X_FCN3(k1, :)';    % (7x1)

    v1 = W3*x1;
    y1 = Sigmoid(v1);

    v2 = W4*y1;
    y2 = Sigmoid(v2);

    v3 = W5*y2;         % (30X7)*(7X1)->(30X1)
    y3 = Sigmoid(v3);   % (30X1)

    v4 = W6*y3;         % (60X30)*(30X1)->(60X1)
    y4 = Sigmoid(v4);   % (60X1)

    v5 = W7*y4;         % (120X60)*(60X1)->(120X1)
    y5 = (Sigmoid(v5)); % (120X1)

    V = W8*y5;          % (784X120)*(120X1)->(784X1)

    Y = reshape(Sigmoid(V), [28 28]);   % (28x28)

    %결과, 출력 한장 한장 보고 싶을때
    %d1 = reshape(D(k1, :)', [28 28]);  % (28x28)


Y1 = imresize(Y, [56, 56]);
subplot(10, 30, k1);
imshow(Y1);
end

%결과, 출력 한장 한장 보고 싶을때

%figure;
%subplot(1, 2, 1);
%imshow(Y);
%title('출력');
%subplot(1, 2, 2);
%imshow(d1);
%title('정답');
