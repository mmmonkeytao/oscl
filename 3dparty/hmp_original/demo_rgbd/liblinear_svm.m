%clear all;

addpath('../liblinear-1.96/matlab');
%addpath('../liblinear-1.5-dense-float/matlab');

%load('rgbdfea_rgb_second.mat');

%load testinstance_eval.mat;
category = 1;
if category
   trail = 1;
   for i = 1:trail
%        % generate training and test samples
%        ttrainindex = [];
%        ttestindex = [];
%        labelnum = unique(rgbdclabel);
%        for j = 1:length(labelnum)
%            trainindex = find(rgbdclabel == labelnum(j));
%            rgbdilabel_unique = unique((trainindex));
%            % perm = randperm(length(rgbdilabel_unique));
%            subindex = find(rgbdilabel(trainindex) == rgbdilabel_unique(testinstance(i,j)));
%            testindex = trainindex(subindex);
%            trainindex(subindex) = [];
%            ttrainindex = [ttrainindex trainindex];
%            ttestindex = [ttestindex testindex];
%        end
% 
%        % train linear SVM
%        trainfea = rgbdfea(:,ttrainindex)';
%        trainlabel = rgbdclabel(ttrainindex)'; % take category label
%        testfea = rgbdfea(:,ttestindex)';
%        testlabel = rgbdclabel(ttestindex)'; % take category label

       %        % train linear SVM
       %idx =randperm(1000);
       %ttrainindex = 1:7112; %idx(1:100);
       %ttestindex = 7113:41876;%idx(101:1000);
       %trainfea = rgbdfea(ttrainindex,:);
       %trainlabel = rgbdclabel(ttrainindex); % take category label
       %testfea = rgbdfea(ttestindex,:);
       %testlabel = rgbdclabel(ttestindex); % take category label
       %classify with liblinear
       %clear rgbdfea rgbdclabel;
       
       lc = 10;
       option = ['-s 1 -c ' num2str(lc)];
       if isunix
           %trainfea
           % dense float inputs
           model = train(double(trainlabel),sparse(double(trainfea)),option);
           % compute classification accuracy
           [predictlabel, accuracy, decvalues] = predict(double(testlabel), sparse(double(testfea)), model);
       else
           % sparse double inputs
           trainfea = sparse(double(trainfea));
           testfea = sparse(double(testfea));
           model = train(trainlabel,trainfea,option);
           % compute classification accuracy
           [predictlabel, accuracy, decvalues] = predict(testlabel, testfea, model);
       end
       % compute classification accuracy
       %[predictlabel, accuracy, decvalues] = predict(double(testlabel), sparse(double(testfea)), model);
       acc_c(i,1) = mean(double(predictlabel) == double(testlabel));
       save(['./rgbdeval_rgb_acc_c_' num2str(5) 'x' num2str(5) '_second.mat'],'acc_c', 'predictlabel', 'testlabel');

       % print and save results
       disp(['Accuracy of Liblinear is ' num2str(mean(acc_c))]);
   end
end

instance = 1;
if instance

   % generate training and test indexes
   indextrain = 1:length(rgbdilabel);
   indextest = find(rgbdvlabel == 2);
   indextrain(indextest) = [];

   % generate training and test samples
   trainfea = rgbdfea(:, indextrain)';
   trainlabel = rgbdilabel(:, indextrain)';
   testfea = rgbdfea(:, indextest)';
   testlabel = rgbdilabel(:, indextest)';

   disp('Performing liblinear ... ...');
   lc = 10;
   % classify with liblinear
   option = ['-s 1 -c ' num2str(lc)];
   if isunix
       % dense float inputs
       model = train(trainlabel,trainfea,option,'','trainfea');
       % compute classification accuracy
       [predictlabel, accuracy, decvalues] = predict(testlabel, testfea, model);
   else
       % sparse double inputs
       trainfea = sparse(double(trainfea));
       testfea = sparse(double(testfea));
       model = train(trainlabel,trainfea,option);
       % compute classification accuracy
       [predictlabel, accuracy, decvalues] = predict(testlabel, testfea, model);
   end
   acc_i = mean(predictlabel == testlabel);
   save(['./rgbdeval_rgb_acc_i_' num2str(dic_first.patchsize) 'x' num2str(dic_first.patchsize) '_second.mat'],'acc_i', 'predictlabel', 'testlabel');

   % print and save classification accuracy
   disp(['Accuracy of Liblinear is ' num2str(mean(acc_i))]);
end