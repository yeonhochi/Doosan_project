

% %-----------------------------입력 데이터 생성----------------------------%
% 
% 3bit(연산자) + 4bit(숫자) + 4bit(숫자) 순서

matrix11bit = zeros(300, 11);
index = 1;

%FCN1 출력
bit4=[0 0 0 0; %0
     0 0 0 1;   %1
     0 0 1 0;   %2
     0 0 1 1;   %3
     0 1 0 0;   %4
     0 1 0 1;   %5
     0 1 1 0;   %6
     0 1 1 1;   %7
     1 0 0 0;   %8
     1 0 0 1;   %9
     ]

%CNN3출력
bit3=[ 1 0 0;   %더하기
       0 1 0;   %빼기
       0 0 1;   %곱하기
      ]


%11bit 생성 ->연산자(3bit),0~9까지 
for i = 1:3
    for j = 1:10
        for k = 1:10
            % 3*3 행렬 한 행
            CNN3_3bit = bit3(i, :);

            % 10*4 행렬 두 개 각각 한 행
            CNN1_4bit = bit4(k, :);
            CNN2_4bit = bit4(j, :);

            % 행렬 합치기
            matrix11bit(index, :) = [CNN3_3bit, CNN1_4bit, CNN2_4bit];
            % 연산자(3bit)+

            % 행 인덱스 증가
            index = index + 1;
        end
    end
end


%입력데이터 11bit
X = matrix11bit;


%-----------------------------목표 데이터 생성----------------------------%


%덧셈연산
plusresult  = zeros(100, 8);

% 인덱스 초기화
index = 1;
for i = 0:9
    for j = 0:9
        result_binary_addition = dec2bin(i + j, 8);%10진수 2진수 8bit으로 변경
        result_binary_array_addition = result_binary_addition - '0';
        plusresult(index, :) = result_binary_array_addition;
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

        %음수일경우 1로 시작
        if(result<0)
            result = -result;
            result_binary_addition = dec2bin(result, 8); %10진수 2진수 8bit으로 변경
            result_binary_array_addition = result_binary_addition - '0';
            subresult(index, :) = result_binary_array_addition;
            subresult(index, 1)=1;
        else
        %양수는 8bit생성 되는 거 그대로
        result_binary_addition = dec2bin(result, 8);
        result_binary_array_addition = result_binary_addition - '0';
        subresult(index, :) = result_binary_array_addition;
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
        result_binary_addition = dec2bin(i * j, 8);%10진수 2진수 8bit으로 변경
        result_binary_array_addition = result_binary_addition - '0';
        mulresult(index, :) = result_binary_array_addition;
        index = index + 1;

    end
end


D = zeros(300, 8);

for i=1:100
D(i,:)=plusresult(i,:);
end

for i=101:200
D(i,:)=subresult(i-100,:);
end

for i=201:300
D(i,:)=mulresult(i-200,:);
end

% 
% %-------------------------------------------------------------------------%
% 
% 
% % 가중치 행렬 초기화
% W1 = 2*rand(50, 11) - 1;     % (50x11)
% W2 = 2*rand(90, 50) - 1;     % (90x50)
% W3 = 2*rand(150, 90) - 1;    % (150x90)
% W4 = 2*rand(8, 150) - 1;     % (8x150)
% 
% for epoch = 1:1000 %훈련 데이터셋을 10,000번 반복
%     epoch
%  % 훈련 과정
%     [W1, W2, W3, W4] = FCN2F(W1, W2, W3, W4, X, D);
% end
% save('FCN2 result.mat')


load('FCN2 result.mat')


N = 300;             % 데이터 수 
acc=0;
for k = 1:N
x = X(k, :)';       % (11x1)

v1 = W1*x;          % (50x11)*(11X1)->(50X1)
y1 = Sigmoid(v1);   % (50X1)

v2 = W2*y1;         % (90x50)*(50X1)->(90X1)
y2 = Sigmoid(v2);   % (90X1)

v3 = W3*y2;         % (150x90)*(90X1)->(150X1)
y3 = Sigmoid(v3);   % (150X1)

v = W4*y3;          % (8x150)*(150X1)->(8X1)
y = round(Sigmoid(v));  % (8X1)

y=y';                   % (1X8)

%정확도 계산
if(y==D(k,:))
    acc=acc+1;
end
fprintf(1, '%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\n', y);
end
%정확도 출력
acc = acc / N;
fprintf('정확도 %.2f%%\n', acc*100);
