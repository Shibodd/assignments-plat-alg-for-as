function fn = gauss_disturb_factory(mu, sigma)
    function y = disturb_fn(~, x)
        % Assume that measurements are independent to avoid
        % having to implement a multivariate normal distribution
        y = x + (mu + randn(size(x)) .* sigma);
    end
    fn = @disturb_fn;
end

