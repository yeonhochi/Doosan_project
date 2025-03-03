
%-----------------------------입력 데이터 생성----------------------------%
%                  

%덧셈연산
plusresult  = zeros(100, 8);

% 인덱스 초기화
index = 1;
for i = 0:9
    for j = 0:9
        result2binary = dec2bin(i + j, 8);
        result2binary_last = result2binary - '0';
        plusresult(index, :) = result2binary_last;
        index = index + 1;

    end
end



%뺄셈연산
subresult = zeros(100, 8);

% 인덱스 초기화
index = 1;
for i = 0:9
    for j = 0:9
        % 덧셈 결과 계산 및 변환
        result=j-i;

        if(result<0)
            result = -result;
            result2binary = dec2bin(result, 8);
            result2binary_last = result2binary - '0';
            subresult(index, :) = result2binary_last;
            subresult(index, 1)=1;
        else
        result2binary = dec2bin(result, 8);
        result2binary_last = result2binary - '0';
        subresult(index, :) = result2binary_last;
        end

        index = index + 1;

    end
end



%곱셈연산
mulresult = zeros(100, 8);

% 인덱스 초기화
index = 1;

for i = 0:9
    for j = 0:9
        % 덧셈 결과 계산 및 변환
        result2binary = dec2bin(i * j, 8);
        result2binary_last = result2binary - '0';
        mulresult(index, :) = result2binary_last;
        index = index + 1;

    end
end

%모든 연산에 대한 경우의 수가 들어간 300x8행렬
x = zeros(300, 8);

%덧셈, 뺄셈, 곱하기 행렬 합치기
for i=1:100
x(i,:)=plusresult(i,:);
end

for i=101:200
x(i,:)=subresult(i-100,:);
end

for i=201:300
x(i,:)=mulresult(i-200,:);
end

N = 300;             % 데이터 수 

for k = 1:N
%목표 데이터(모든 연산 결과에 대한 이미지 300장(784X1))
D(k,:) = reshape(rgb2gray(im2double(imread( ...
    ['C:\Users\82102\Desktop\인공지능 세그먼트 이미지\', num2str(k-1), '.png']))), [1 784]);

end

W3 = 2*rand(300, 8) - 1;      % (300x8)
W4 = 2*rand(400, 300) - 1;    % (400x300)
W5 = 2*rand(500, 400) - 1;    % (500x400)
W6 = 2*rand(600, 500) - 1;   % (600x500)
W7 = 2*rand(700, 600) - 1;  % (700x600)
W8 = 2*rand(784, 700) - 1; % (784x700)

for epoch=1:5000 %훈련 5000번 반복  
    epoch

    [W3, W4, W5, W6, W7, W8] = FCN3F(W3, W4, W5, W6, W7, W8, X, D);
end

save('FCN3 result.mat');

load('FCN3 result.mat');
acc=0;
N = 300;
for k=1:N
    x = X(k, :)';       %(8x1)

    v1 = W3*x;          %(300X8)*(8x1)->(300X1)
    y1 = Sigmoid(v1);   %(300X1)

    v2 = W4*y1;         %(400X300)*(300X1)->(300X1)
    y2 = Sigmoid(v2);   %(400X1)

    v3 = W5*y2;         % (500X400)*(400X1)->(500X1)
    y3 = Sigmoid(v3);   % (500X1)

    v4 = W6*y3;         % (600X500)*(500X1)->(600X1)
    y4 = Sigmoid(v4);   % (600X1)

    v5 = W7*y4;         % (700X600)*(600X1)->(700X1)
    y5 = (Sigmoid(v5)); % (700X1)

    V = W8*y5;          % (784X700)*(700X1)->(784X1)

    y = reshape(Sigmoid(V), [28 28]);   % (28x28)

    d1 = reshape(D(k, :)', [28 28]);  % (28x28)


 figure;
 subplot(1, 2, 1);
 imshow(y);
 title('출력');
 subplot(1, 2, 2);
 imshow(d1);
 title('정답');

end

