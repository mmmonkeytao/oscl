function omp_pooling = omp_pooling_layer1(fea_first, dic_first, encoder_first)
% Pool sparse codes
% Written by Liefeng Bo on March 2012

% Batch Orthogonal Matching Pursuit
omp_codes = omp_coding_layer1(fea_first, dic_first, encoder_first);
% max pooling
% py = floor(size(omp_codes,1)/encoder_first.pooling);
% px = floor(size(omp_codes,2)/encoder_first.pooling);
% omp_pooling = zeros(py, px, size(omp_codes,3));
% for i = 1:py
%     for j = 1:px
%         ind_y = encoder_first.pooling*(i-1)+(1:encoder_first.pooling);
%         ind_x = encoder_first.pooling*(j-1)+(1:encoder_first.pooling);
%         codes = omp_codes(ind_y,ind_x,:);
%         codes = reshape(codes,encoder_first.pooling^2, size(codes,3));
%         omp_pooling(i,j,:) = max(codes,[],1);
%     end
% end

% spatial max pooling layer1
% batch orthogonal matching pursuit
% spatial pyramid max pooling

fea_width = size(omp_codes, 2);
fea_height = size(omp_codes, 1);

sample_rows = floor((fea_height - 16)/4) + 1; 
sample_cols = floor((fea_width - 16)/4) + 1; 

patch_idx = 1:1:16;
patch_idy = 1:1:16;

pooling = [4 2 1];
pgrid = pooling.^2;
wordsnum = size(omp_codes, 3);
sgrid = 21;

for j = 1:sample_cols
    for i = 1:sample_rows
        sub_idy = patch_idy + (j-1)*4;
        sub_idx = patch_idx + (i-1)*4;
        
        patch = omp_codes(sub_idx, sub_idy,:);
        omp_fea = reshape(patch,size(patch,1)*size(patch,2), size(patch,3));
        omp_fea = omp_fea';

        [fea_x, fea_y] =  meshgrid(1:16, 1:16);
        fea_x = fea_x(:);
        fea_y = fea_y(:);
        imfea = zeros(sgrid*wordsnum,1);
        for s = 1:length(pooling)
           wleng = 16/pooling(s);
           hleng = 16/pooling(s);
           xgrid = ceil(fea_x/wleng);
           ygrid = ceil(fea_y/hleng);
           allgrid = (ygrid -1 )*pooling(s) + xgrid;
           pimfea = zeros(pgrid(s)*wordsnum,1);
           for t = 1:pgrid(s)
               % find features localized in the corresponding pyramid grid
               ind = find(allgrid == t);
               if length(ind)
                  pimfea((t-1)*wordsnum+(1:wordsnum)) = max(omp_fea(:, ind), [], 2);
               else
                  pimfea((t-1)*wordsnum+(1:wordsnum)) = 0;
               end
           end
           imfea(sum(pgrid(1:s-1))*wordsnum + (1:pgrid(s)*wordsnum),1) = pimfea;
        end

        omp_pooling(i, j, :) = imfea./(sqrt(sum(imfea.^2))+eps);
            
    end 
end