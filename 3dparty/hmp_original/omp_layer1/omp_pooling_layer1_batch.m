function omp_pooling_batch_layer1(fea_first, dic_first, encoder_first)
% Pool sparse codes for all images
% Written by Liefeng Bo on March 2012

if exist(fea_first.savedir,'dir')
    ;
else
    mkdir(fea_first.savedir);
end

disp('Orthogonal Matching Pursuit Encoder ...');
for i = 1:length(fea_first.feapath)
    
    % read raw data
    switch fea_first.type
    case 'rgb'
       im = imread(fea_first.feapath{i});
       if size(im,3) == 1
          im = color(im);
       end
       im = im2double(im);
    case 'rgbcrop'
       im = imread(fea_first.feapath{i});
       if size(im,3) == 1
          im = color(im);
       end
       im = mask_im(im);
    case 'gray'
       im = imread(fea_first.feapath{i});
       if size(im,3) == 3
          im = rgb2gray(im);
       end
       im = im2double(im);
    case 'graycrop'
       im = imread(fea_first.feapath{i});
       if size(im,3) == 3
          im = rgb2gray(im);
       end
       im = mask_im(im);
    case 'depth'
       im = imread(fea_first.feapath{i});
       im = double(im);
       threshold = 1200;
       im(im > threshold) = threshold;
       im = im/threshold;
    case 'depthcrop'
       im = imread(fea_first.feapath{i});
       threshold = 1200;
       im = mask_depth(im, threshold);
    case 'normal'
       %load(fea_first.feapath{i});
       normal = depthtonormal(fea_first.feapath{i});
       im = normal;
    otherwise
       disp('unknown data type');
    end

    % resize images
    if fea_first.resizetag
       im_h = size(im,1);
       im_w = size(im,2);
       if max(im_h, im_w) > fea_first.maxsize,
          im = imresize(im, fea_first.maxsize/max(im_h, im_w), 'bicubic');
       end
       if min(im_h, im_w) < fea_first.minsize,
          im = imresize(im, fea_first.minsize/min(im_h, im_w), 'bicubic');
       end;
    end
    fea_first.fea = im;   
 
    % record feature extraction time
    tic;
    omp_pooling = omp_pooling_layer1(fea_first, dic_first, encoder_first);
    time = toc; 

    % save pooled features
    save([fea_first.savedir '/' sprintf('%06d',i)], 'omp_pooling');

    % print feature extraction information
    ind = find(fea_first.feapath{i} == '/');
    fprintf('Image ID %s: width= %d, height= %d, time %f\n', fea_first.feapath{i}(ind(end)+1:end), size(omp_pooling,2), size(omp_pooling,1), time);
end;

