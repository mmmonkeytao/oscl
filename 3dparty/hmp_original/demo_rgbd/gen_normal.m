% Convert depth values to surface normals
% Written by Liefeng Bo on March 2012

clear;

addpath('../helpfun');
imdir = '../../../data/sample4dicfull/';
imsubdir = dir_bo(imdir);
it = 0;
for i = 1:length(imsubdir)
    [rgbdilabel, impath] = get_im_label([imdir imsubdir(i).name '/'], '_crop.png');
    [rgbdilabel, depthpath] = get_im_label([imdir imsubdir(i).name '/'],'_depthcrop.png');
    for j = 1:length(impath)
        it = it + 1;
        im = imread(depthpath{j});
        im = double(im);
        topleft = fliplr(load([depthpath{j}(1:end-13) 'loc.txt']));
        pcloud = depthtocloud(im, topleft);
        pcloud = pcloud./1000; % normalized to meter
        normal = pcnormal(pcloud);
        save([impath{j}(1:end-8) 'normal.mat'],'normal');
        if mod(it,100) == 1;
           disp(['Current Iteration is ' num2str(it)]);
        end
    end
end

