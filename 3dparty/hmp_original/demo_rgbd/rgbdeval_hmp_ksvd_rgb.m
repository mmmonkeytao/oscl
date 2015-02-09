% Category and instance recognition using the rgb channel
% Written by Liefeng Bo on Jan 2013

clear;

% add paths
addpath('../liblinear-1.51/matlab');
addpath('../liblinear-1.5-dense-float/matlab');
addpath('../helpfun');
addpath('../omp_layer1');
addpath('../omp_layer2');
addpath('../learn_layer1');
addpath('../learn_layer2');
addpath(genpath('../ksvdbox/'));

% Hierarchical Matching Pursuit for Object Recognition
% tic;
% dic1 = load('rgbdeval_dic_5x5_graycrop.mat');
% dic1 = dic1.dic;

% compute the paths of datasets
imdir = '../../../data/sample4dicfull/';
%imdir = '../../../data/rgbdsubset/';
imsubdir = dir_bo(imdir);
impath = [];
rgbdclabel = [];
rgbdilabel = [];
rgbdvlabel = [];
subsample = 1;
label_num = 0;
for i = 1:length(imsubdir)
    [rgbdilabel_tmp, impath_tmp] = get_im_label([imdir imsubdir(i).name '/'], '_crop.png');
    for j = 1:length(impath_tmp)
        ind = find(impath_tmp{j} == '_');
        rgbdvlabel_tmp(1,j) = str2num(impath_tmp{j}(ind(end-2)+1));
    end

    it = 0;
    for j = 1:subsample:length(impath_tmp)
        it = it + 1;
        impath_tmp_sub{it} = impath_tmp{j};
    end
    impath = [impath impath_tmp_sub];
    rgbdclabel = [rgbdclabel i*ones(1,length(impath_tmp_sub))];
    rgbdilabel = [rgbdilabel rgbdilabel_tmp(1:subsample:end)+label_num];
    rgbdvlabel = [rgbdvlabel rgbdvlabel_tmp(1:subsample:end)];
    label_num = label_num + length(unique(rgbdilabel_tmp));
    clear impath_tmp_sub rgbdvlabel_tmp;
end

% initialize the paramters of feature (data)
type{1} = 'graycrop'; type{2} = 'rgbcrop';
dicsize{1} = 75; dicsize{2} = 150;
saveroot = 'hmpfea/';
rgbdfea = [];
for i = 1:length(type)
    % first layer of HMP
    % initialize the parameters of dictionary
    dic_first.dicsize = dicsize{i};
    dic_first.patchsize = 5;
    dic_first.samplenum = 20;

    % initialize the paramters of feature (data)
    fea_first.feapath = impath;
    fea_first.type = type{i};
    fea_first.resizetag = 1;
    fea_first.maxsize = 300;
    fea_first.minsize = 50;
    fea_first.savedir = [saveroot '/rgb_hmp_fea_' num2str(dic_first.patchsize) 'x' num2str(dic_first.patchsize) '_' fea_first.type];
    mkdir_bo(fea_first.savedir);
    rgbdfeapath = get_fea_path(fea_first.savedir);
    if ~length(rgbdfeapath)
       % dictionary learning
       dic = ksvd_learn_layer1(fea_first, dic_first);
       dic_first.dic = dic;
       save(['./rgb51_dic_' num2str(dic_first.patchsize) 'x' num2str(dic_first.patchsize) '_' fea_first.type '.mat'],'dic');
       % initialize the parameters of encoder
       encoder_first.coding = 'omp';
       encoder_first.pooling = 4;
       encoder_first.sparsity = 4;
       % orthogonal matching pursuit encoder for the first layer
       omp_pooling_layer1_batch(fea_first, dic_first, encoder_first);
       rgbdfeapath = get_fea_path(fea_first.savedir);
    end

    % second layer of HMP
    % initialize the parameters of dictionary
    dic_second.dicsize = 1000;
    dic_second.patchsize = 4;
    dic_second.samplenum = 20;
    % initialize the paramters of features
    fea_second.feapath = rgbdfeapath;
    % dictionary learning
    dic = ksvd_learn_layer2(fea_second, dic_second);
    dic_second.dic = dic;
    save(['./rgb51_dic_' num2str(dic_first.patchsize) 'x' num2str(dic_first.patchsize) '_2nd_' fea_first.type '.mat'],'dic');
    % initialize the parameters of encoder
    encoder_second.coding = 'omp';
    encoder_second.fea = 'second+first';
    % encoder_second.fea = 'second+first'; % for the higher accuracy
    encoder_second.pooling = [1 2 3];
    encoder_second.sparsity = 10;
    % orthogonal matching pursuit encoder for the second layer
    rgbdfea_one = omp_pooling_layer2_batch(fea_second, dic_second, encoder_second);
    rgbdfea = [rgbdfea; single(rgbdfea_one)];
end

save -v7.3 rgbdfea_rgb_second.mat rgbdfea rgbdclabel rgbdilabel rgbdvlabel;

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
           rgbdilabel_unique = unique((trainindex));
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
       testfea = rgbdfea(:,ttestindex)';
       testlabel = rgbdclabel(ttestindex)'; % take category label

       % classify with liblinear
       lc = 10;
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
       % compute classification accuracy
       [predictlabel, accuracy, decvalues] = predict(testlabel, testfea, model);
       acc_c(i,1) = mean(predictlabel == testlabel);
       save(['./rgbdeval_rgb_acc_c_' num2str(dic_first.patchsize) 'x' num2str(dic_first.patchsize) '_second.mat'],'acc_c', 'predictlabel', 'testlabel');

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


