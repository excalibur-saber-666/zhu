function [Xc, PK, Xerr, diagnostics] = cooperative_range_ekf_update(varargin)
%COOPERATIVE_RANGE_EKF_UPDATE Public EKF name for cooperative range updates.
%   The range observation is nonlinear; its current-epoch Jacobian is formed
%   by the maintained implementation in cooperative_range_kf_update.  Keep
%   that legacy helper available for compatibility while new experiments call
%   this scientifically explicit EKF entry point.

[Xc, PK, Xerr, diagnostics] = cooperative_range_kf_update(varargin{:});
end
