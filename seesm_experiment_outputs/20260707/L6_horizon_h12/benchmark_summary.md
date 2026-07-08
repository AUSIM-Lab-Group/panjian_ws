| scenario | context_level | method | success_rate | collision_rate | D_min | min_h_SEESM | semantic_violation_ratio | feasibility_rate | path_length | p95_solve_time_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| head_on | context_bl | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0003 | -0.0003 | 0.0250 | 1.0000 | 4.6692 | 9.3378 |
| head_on | context_bl | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.1050 | 0.1125 | 0.0000 | 1.0000 | 4.7289 | 11.9713 |
| head_on | context_bl | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.1916 | 0.1273 | 0.0000 | 1.0000 | 4.8285 | 7.1757 |
| head_on | context_bl | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.1915 | 0.1273 | 0.0000 | 1.0000 | 4.8288 | 8.0155 |
| crossing | context_bl | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0474 | -0.0474 | 0.0833 | 1.0000 | 5.0048 | 30.7708 |
| crossing | context_bl | EESM-MPC-ECBF | 0.0000 | 1.0000 | -0.0001 | -0.0001 | 0.0167 | 1.0000 | 5.2742 | 39.4898 |
| crossing | context_bl | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.0867 | -0.0779 | 0.0417 | 1.0000 | 6.1958 | 42.2672 |
| crossing | context_bl | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.0844 | 0.0500 | 0.0000 | 1.0000 | 6.0733 | 41.0293 |
| local_crowding | context_bl | Standard MPC-CBF | 1.0000 | 0.0000 | 0.0432 | 0.0432 | 0.0000 | 1.0000 | 4.5214 | 6.9153 |
| local_crowding | context_bl | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.2503 | 0.2510 | 0.0000 | 1.0000 | 4.5039 | 6.4581 |
| local_crowding | context_bl | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.3463 | 0.1910 | 0.0000 | 1.0000 | 4.4806 | 6.4903 |
| local_crowding | context_bl | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.3422 | 0.1877 | 0.0000 | 1.0000 | 4.4845 | 5.7983 |
| head_on | context_int | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0003 | -0.0003 | 0.0250 | 1.0000 | 4.6692 | 9.8205 |
| head_on | context_int | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.1050 | 0.1125 | 0.0000 | 1.0000 | 4.7289 | 6.2866 |
| head_on | context_int | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.5360 | 0.0589 | 0.0000 | 1.0000 | 5.2100 | 6.1914 |
| head_on | context_int | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.3128 | 0.0878 | 0.0000 | 1.0000 | 4.9162 | 7.6585 |
| crossing | context_int | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0474 | -0.0474 | 0.0833 | 1.0000 | 5.0048 | 37.0032 |
| crossing | context_int | EESM-MPC-ECBF | 0.0000 | 1.0000 | -0.0001 | -0.0001 | 0.0167 | 1.0000 | 5.2742 | 46.3073 |
| crossing | context_int | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.0276 | -0.4521 | 0.2583 | 1.0000 | 7.2283 | 61.7937 |
| crossing | context_int | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.0910 | 0.0500 | 0.0000 | 1.0000 | 6.5393 | 46.3919 |
| local_crowding | context_int | Standard MPC-CBF | 1.0000 | 0.0000 | 0.0432 | 0.0432 | 0.0000 | 1.0000 | 4.5214 | 6.3476 |
| local_crowding | context_int | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.2503 | 0.2510 | 0.0000 | 1.0000 | 4.5039 | 6.5470 |
| local_crowding | context_int | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.5310 | -0.0353 | 0.0083 | 1.0000 | 4.5767 | 6.6278 |
| local_crowding | context_int | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.3925 | 0.1326 | 0.0000 | 1.0000 | 4.4930 | 6.3703 |
| head_on | context_ext | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0003 | -0.0003 | 0.0250 | 1.0000 | 4.6692 | 5.8447 |
| head_on | context_ext | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.1050 | 0.1125 | 0.0000 | 1.0000 | 4.7289 | 5.6777 |
| head_on | context_ext | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.6035 | 0.0702 | 0.0000 | 1.0000 | 5.3004 | 5.4708 |
| head_on | context_ext | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.2998 | 0.0500 | 0.0000 | 1.0000 | 4.9129 | 5.3748 |
| crossing | context_ext | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0474 | -0.0474 | 0.0833 | 1.0000 | 5.0048 | 30.6334 |
| crossing | context_ext | EESM-MPC-ECBF | 0.0000 | 1.0000 | -0.0001 | -0.0001 | 0.0167 | 1.0000 | 5.2742 | 36.9279 |
| crossing | context_ext | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.0617 | -0.5188 | 0.2833 | 1.0000 | 7.3925 | 54.2552 |
| crossing | context_ext | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.1075 | 0.0500 | 0.0000 | 1.0000 | 6.3328 | 54.0211 |
| local_crowding | context_ext | Standard MPC-CBF | 1.0000 | 0.0000 | 0.0432 | 0.0432 | 0.0000 | 1.0000 | 4.5214 | 6.5139 |
| local_crowding | context_ext | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.2503 | 0.2510 | 0.0000 | 1.0000 | 4.5039 | 6.1671 |
| local_crowding | context_ext | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.6973 | -0.0914 | 0.0250 | 1.0000 | 4.6925 | 7.4296 |
| local_crowding | context_ext | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.3418 | 0.1418 | 0.0000 | 1.0000 | 4.4850 | 5.5146 |
