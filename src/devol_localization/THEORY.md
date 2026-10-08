# Localization theory and references

Where each step of the EKF, the particle filter and the hybrid comes from, and where the code
departs from the textbook form. The code's docstrings and comments cite the same sources; the
keys in brackets match `docs/Reasoning Under Uncertainty/Annotated Outline/ref.bib`, and the
BibTeX for the sources not yet in that file is at the end.

"PR" is Thrun, Burgard and Fox, *Probabilistic Robotics*, MIT Press, 2005
[thrun2005probabilistic]. Table numbers refer to its algorithm listings.

## Shared models

| Step | Code | Source |
|---|---|---|
| Odometry increment as (rot1, trans, rot2) | `ekf_core.odometry_delta`, `particle_filter.odometry_delta` | PR Section 5.4, Tables 5.5 and 5.6 (lines 2-4) |
| Distance-to-nearest-obstacle field | `particle_filter.LikelihoodField`, `scan_matcher.DistanceField` | PR Section 6.4 (likelihood fields); Euclidean distance transform from SciPy |
| Bayes filter structure (predict with u_t, correct with z_t) | both filters | PR Section 2.4, Table 2.1 |

## Extended Kalman filter (`ekf_core.py`, `ekf_pipeline.py`, `scan_matcher.py`)

EKF localization against a known map [leonard1991mobile]; PR Section 7.4.

| Step | Code | Source |
|---|---|---|
| Gaussian prior N(mu_0, Sigma_0) | `PoseEKF.reset` | PR Section 3.2 |
| Mean prediction mu_bar = g(u, mu) | `PoseEKF.predict` | PR Table 3.3 line 2; odometry model of PR Section 5.4 |
| Covariance prediction G Sigma G^T + V M V^T | `PoseEKF.predict` | PR Table 3.3 line 3; G, V, M as in PR Table 7.2 (there for the velocity model) |
| Measurement model: scan to pose observation | `ScanMatcher.match` | Correlative search [olson2009correlative]; Levenberg-Marquardt on the distance field [nocedal2006numerical], bilinear field interpolation as in [kohlbrecher2011flexible] |
| Measurement covariance R | `ScanMatcher.match` | Gauss-Newton sigma^2 (J^T J)^-1, inflated and floored because it is overconfident [censi2007accurate] |
| Innovation, S = Sigma_bar + R, Mahalanobis gate | `PoseEKF.innovation`, `PoseEKF.correct` | PR Table 3.3 line 4; chi-square gate on the normalized innovation squared [barshalom2001estimation] Section 5.4 |
| Gain, mean and covariance update | `PoseEKF.correct` | PR Table 3.3 lines 4-6; Joseph form [barshalom2001estimation] Section 5.2 |
| Search window from the covariance | `EKFPipeline.search_window` | 3-sigma validation region [barshalom2001estimation] |
| Late (out-of-sequence) scans | `EKFPipeline._fuse_at` | Exact reprocessing from a stored state; see [barshalom2002oosm] for the alternatives |
| Global initialization | `global_initial_state` | Moment match of a uniform prior over the free space (one Gaussian cannot represent it; PR Section 7.1) |

Departures from the textbook:
- The control noise variances grow linearly with the increment (alpha * |delta|), not with its
  square as in PR Table 5.6, so the accumulated uncertainty does not depend on the odometry rate.
- The observation is a full pose from the scan matcher, so h(x) = x and H = I, instead of
  PR Table 7.2's landmark range-bearing model; the scan matcher does the data association.
- Covariance inflation while matches keep failing (`EKFPipeline._match_and_fuse`) is a
  heuristic of this implementation, not from the references.

## Particle filter (`particle_filter.py`, `pf_localization_node.py`)

Bootstrap / SIR filter [gordon1993novel], [arulampalam2002tutorial]; Monte Carlo localization
[dellaert1999monte], [fox1999monte]; PR Tables 4.3 and 8.2.

| Step | Code | Source |
|---|---|---|
| Initialization, tracking (Gaussian) | `ParticleFilter.init_gaussian` | PR Section 7.1 (position tracking) |
| Initialization, global (uniform over free space) | `ParticleFilter.init_uniform` | PR Section 8.3.2 |
| Prediction: sample the motion model per particle | `ParticleFilter.predict` | PR Table 5.6 (sample_motion_model_odometry), Tables 4.3 and 8.2 |
| Correction: likelihood field weight | `ParticleFilter.update` | PR Table 6.3 (likelihood_field_range_finder_model), Section 6.4 |
| Importance weights w_t = w_{t-1} p(z_t \| x_t), normalized | `ParticleFilter.update` | Sequential importance sampling with the motion model as proposal [arulampalam2002tutorial], PR Table 4.3 |
| Effective sample size N_eff = 1 / sum w^2 | `ParticleFilter.effective_sample_size` | [arulampalam2002tutorial] |
| Selective resampling when N_eff < 0.5 N | `ParticleFilter.resample_if_needed` | PR Section 4.3.4 |
| Low-variance (systematic) resampling | `ParticleFilter.resample` | PR Table 4.4 |
| Random-particle injection with w_slow, w_fast | `ParticleFilter.update`, `injection_probability`, `resample` | Augmented MCL, PR Section 8.3.5, Table 8.3; [thrun2001robust] |
| Pose estimate: weighted mean, circular mean for yaw, sample covariance | `ParticleFilter.estimate` | [mardia2000directional] Chapter 2 |

Departures from the textbook:
- Motion noise is in std form (std = alpha1 |rot| + alpha2 |trans|, ...) where PR Table 5.6 passes
  the variance alpha1 rot^2 + alpha2 trans^2 to `sample()`. Both scale the std with the increment;
  the alpha values are not interchangeable.
- Increments under `min_translation` get their noise as a pure rotation (rot1 = 0), as ROS AMCL
  does, because the heading of a sub-centimetre creep is arbitrary.
- Only `max_beams` evenly spaced beams are used, and the beam Gaussian is not truncated to
  [0, z_max]. PR Chapter 6 notes that treating every beam of a dense scan as independent makes
  the likelihood overconfident; subsampling tempers that.
- Augmented MCL's w_avg is the mean over particles of the per-beam geometric mean likelihood,
  not of the raw product, so it is comparable between scans with different numbers of valid
  beams. Resampling also runs whenever the injection probability exceeds 1%.
- The filter updates only after the robot has moved (`update_min_d`, `update_min_a`), as AMCL does.

## Hybrid (`hybrid.py`, `hybrid_supervisor.py`)

The EKF publishes; the particle filter runs alongside and re-seeds the EKF after a kidnap. Both
filters are unchanged; the hybrid adds only the switching rule.

| Step | Code | Source |
|---|---|---|
| Why pair them | module docstring | Kalman localization is the most accurate while tracking, MCL recovers from displacement [gutmann2002experimental]; one Gaussian cannot represent a kidnapped robot's belief (PR Section 7.1) |
| Disagreement test | `KidnapMonitor.mahalanobis2` | Chi-square test on the difference of two Gaussian estimates [barshalom2001estimation] |
| Scan tie-break and fit tests | `scan_log_likelihood` | The particle filter's own measurement model (PR Table 6.3) |
| New EKF prior from the particle set | `KidnapMonitor.reseed_prior` | Moment matching of the particle set; resetting from the evidence as in sensor resetting localization [lenser2000sensor] |

The confidence limits, covariance floors and confirmation count are tuning choices of this
implementation, not from the references.

## References

Already in `ref.bib`:
- [thrun2005probabilistic] S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics*. MIT Press, 2005.
- [leonard1991mobile] J. J. Leonard, H. F. Durrant-Whyte. Mobile robot localization by tracking geometric beacons. IEEE Trans. Robotics and Automation 7(3), 1991.
- [gordon1993novel] N. J. Gordon, D. J. Salmond, A. F. M. Smith. Novel approach to nonlinear/non-Gaussian Bayesian state estimation. IEE Proc. F 140(2), 1993.
- [arulampalam2002tutorial] M. S. Arulampalam, S. Maskell, N. Gordon, T. Clapp. A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking. IEEE Trans. Signal Processing 50(2), 2002.
- [dellaert1999monte] F. Dellaert, D. Fox, W. Burgard, S. Thrun. Monte Carlo localization for mobile robots. ICRA 1999.
- [fox1999monte] D. Fox, W. Burgard, F. Dellaert, S. Thrun. Monte Carlo localization: efficient position estimation for mobile robots. AAAI 1999.
- [thrun2001robust] S. Thrun, D. Fox, W. Burgard, F. Dellaert. Robust Monte Carlo localization for mobile robots. Artificial Intelligence 128(1-2), 2001.
- [lenser2000sensor] S. Lenser, M. Veloso. Sensor resetting localization for poorly modelled mobile robots. ICRA 2000.
- [gutmann2002experimental] J.-S. Gutmann, D. Fox. An experimental comparison of localization methods continued. IROS 2002.
- [besl1992method] P. J. Besl, N. D. McKay. A method for registration of 3-D shapes. IEEE TPAMI 14(2), 1992.

Not yet in `ref.bib`:

```bibtex
@book{barshalom2001estimation,
  title={Estimation with Applications to Tracking and Navigation},
  author={Bar-Shalom, Yaakov and Li, X. Rong and Kirubarajan, Thiagalingam},
  year={2001},
  publisher={Wiley},
  address={New York}
}

@article{barshalom2002oosm,
  title={Update with Out-of-Sequence Measurements in Tracking: Exact Solution},
  author={Bar-Shalom, Yaakov},
  journal={IEEE Transactions on Aerospace and Electronic Systems},
  volume={38},
  number={3},
  year={2002}
}

@inproceedings{olson2009correlative,
  title={Real-Time Correlative Scan Matching},
  author={Olson, Edwin B.},
  booktitle={Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)},
  year={2009}
}

@inproceedings{kohlbrecher2011flexible,
  title={A Flexible and Scalable {SLAM} System with Full {3D} Motion Estimation},
  author={Kohlbrecher, Stefan and von Stryk, Oskar and Meyer, Johannes and Klingauf, Uwe},
  booktitle={Proceedings of the IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)},
  year={2011}
}

@inproceedings{censi2007accurate,
  title={An Accurate Closed-Form Estimate of {ICP}'s Covariance},
  author={Censi, Andrea},
  booktitle={Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)},
  year={2007}
}

@book{nocedal2006numerical,
  title={Numerical Optimization},
  author={Nocedal, Jorge and Wright, Stephen J.},
  edition={2},
  year={2006},
  publisher={Springer},
  address={New York}
}

@book{mardia2000directional,
  title={Directional Statistics},
  author={Mardia, Kanti V. and Jupp, Peter E.},
  year={2000},
  publisher={Wiley},
  address={Chichester}
}
```
