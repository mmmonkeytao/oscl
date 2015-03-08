% Category and instance recognition using the rgb and depth channels
% Written by Liefeng Bo on Jan 2013

clear;

% add paths
addpath('../liblinear-1.51/matlab');
addpath('../liblinear-1.5-dense-float/matlab');

load rgbdfea_rgb_second.mat
rgbdfea_rgb = rgbdfea;
load rgbdfea_depth_second.mat
if isunix
    rgbdfea = [rgbdfea_rgb; rgbdfea];
else
    rgbdfea = [rgbdfea_rgb; rgbdfea];
    rgbdfea = sparse(double(rgbdfea));
end
clear rgbdfea_rgb;

load testinstance_subset;
category = 1;
if category
   trail = 10;
   for i = 1:trail
       % generate training and test samples
       ttrainindex = [];
       ttestindex = [];
       labelnum = unique(rgbdclabel);
       for j = 1:length(labelnum)
           trainindex = find(rgbdclabel == labelnum(j));
           rgbdilabel_unique = unique(rgbdilabel(trainindex));
           % perm = randperm(length(rgbdilabel_unique));
           subindex = find(rgbdilabel(trainindex) == rgbdilabel_unique(testinstance(i,j)));
           testindex = trainindex(subindex);
           trainindex(subindex) = [];
           ttrainindex = [ttrainindex trainindex];
           ttestindex = [ttestindex testindex];
       end

       % train linear SVM
       trainfea = rgbdfea(:,ttrainindex)';
       trainlabel = rgbdclabel(ttrainindex)'; % take category label
       % compute classification accuracy
       testfea = rgbdfea(:,ttestindex)';
       testlabel = rgbdclabel(ttestindex)'; % take category label
       
       % classify with liblinear
       lc = 10;
       option = ['-s 1 -c ' num2str(lc)];
       if isunix
           model = train(trainlabel,trainfea,option,'','trainfea');
           [predictlabel, accuracy, decvalues] = predict(testlabel, testfea, model);
       else
           % sparse double inputs
           trainfea = sparse(double(trainfea));
           testfea = sparse(double(testfea));
           model = train(trainlabel,trainfea,option);
           % compute classification accuracy
           [predictlabel, accuracy, decvalues] = predict(testlabel, testfea, model);
       end
       acc_c(i,1) = mean(predictlabel == testlabel);
       save(['./rgbdsubset_joint_acc_c_second.mat'],'acc_c', 'predictlabel', 'testlabel');

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
       model = train(trainlabel,trainfea,option,'','trainfea');
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
   save(['./rgbdsubset_joint_acc_i_second.mat'],'acc_i', 'predictlabel', 'testlabel');

   % print and save classification accuracy
   disp(['Accuracy of Liblinear is ' num2str(mean(acc_i))]);
end


