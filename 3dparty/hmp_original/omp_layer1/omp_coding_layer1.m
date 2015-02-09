function omp_codes = omp_coding_layer1(fea_first, dic_first, encoder_first)
% Compute sparse codes by orthogonal matching pursuit
% Written by Liefeng Bo on March 2012

switch encoder_first.coding

    case 'omp'
        % overcomplete basis vectors
        if size(fea_first.fea,3) == 1
           X = im2colstep(fea_first.fea, [dic_first.patchsize dic_first.patchsize], [1 1] );
        else
           X = im2colstep(fea_first.fea, [dic_first.patchsize dic_first.patchsize size(fea_first.fea,3)], [1 1 1] );
        end
        X = remove_dc(X, 'columns');
        omp_codes = omp(dic_first.dic'*X, dic_first.dic'*dic_first.dic, encoder_first.sparsity, 'gammamode', 'full');
        omp_codes = abs(reshape(full(omp_codes'), size(fea_first.fea,1)-dic_first.patchsize+1,size(fea_first.fea,2)-dic_first.patchsize+1, dic_first.dicsize));

    otherwise
        disp('unknown type');
end

