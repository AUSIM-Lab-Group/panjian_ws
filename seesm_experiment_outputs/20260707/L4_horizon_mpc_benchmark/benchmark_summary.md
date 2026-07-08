| scenario | context_level | method | success_rate | collision_rate | D_min | min_h_SEESM | semantic_violation_ratio | feasibility_rate | path_length | p95_solve_time_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| head_on | context_bl | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0003 | -0.0003 | 0.0250 | 1.0000 | 4.6823 | 5.0709 |
| head_on | context_bl | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.0409 | 0.0426 | 0.0000 | 1.0000 | 4.7300 | 5.1936 |
| head_on | context_bl | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.1857 | -0.0505 | 0.0250 | 1.0000 | 4.9600 | 13.1961 |
| head_on | context_bl | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.1838 | 0.0500 | 0.0000 | 1.0000 | 4.9544 | 6.8479 |
| crossing | context_bl | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0377 | -0.0377 | 0.0583 | 1.0000 | 5.1668 | 12.6757 |
| crossing | context_bl | EESM-MPC-ECBF | 0.0000 | 1.0000 | -0.0002 | -0.0050 | 0.0250 | 1.0000 | 5.4472 | 12.8659 |
| crossing | context_bl | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.0682 | -0.0816 | 0.1167 | 1.0000 | 6.4737 | 18.6800 |
| crossing | context_bl | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.0123 | 0.0338 | 0.0000 | 1.0000 | 6.2954 | 16.9033 |
| local_crowding | context_bl | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0001 | -0.0001 | 0.0167 | 1.0000 | 4.5699 | 5.3279 |
| local_crowding | context_bl | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.1788 | 0.1814 | 0.0000 | 1.0000 | 4.5123 | 4.9990 |
| local_crowding | context_bl | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.3089 | 0.0042 | 0.0000 | 1.0000 | 4.5510 | 6.8240 |
| local_crowding | context_bl | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.2826 | 0.1714 | 0.0000 | 1.0000 | 4.5085 | 4.8409 |
| head_on | context_int | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0003 | -0.0003 | 0.0250 | 1.0000 | 4.6823 | 5.1456 |
| head_on | context_int | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.0409 | 0.0426 | 0.0000 | 1.0000 | 4.7300 | 5.2136 |
| head_on | context_int | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.5545 | -0.2229 | 0.0833 | 1.0000 | 5.4234 | 14.0145 |
| head_on | context_int | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.3196 | 0.0946 | 0.0000 | 1.0000 | 4.9926 | 5.5140 |
| crossing | context_int | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0377 | -0.0377 | 0.0583 | 1.0000 | 5.1668 | 12.2036 |
| crossing | context_int | EESM-MPC-ECBF | 0.0000 | 1.0000 | -0.0002 | -0.0050 | 0.0250 | 1.0000 | 5.4472 | 12.6373 |
| crossing | context_int | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.2131 | -0.5121 | 0.2333 | 1.0000 | 7.0359 | 17.6402 |
| crossing | context_int | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.0143 | 0.0379 | 0.0000 | 1.0000 | 6.8468 | 25.5640 |
| local_crowding | context_int | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0001 | -0.0001 | 0.0167 | 1.0000 | 4.5699 | 5.3991 |
| local_crowding | context_int | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.1788 | 0.1814 | 0.0000 | 1.0000 | 4.5123 | 5.0547 |
| local_crowding | context_int | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.5015 | -0.0378 | 0.0167 | 1.0000 | 4.6431 | 6.6752 |
| local_crowding | context_int | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.3380 | 0.0875 | 0.0000 | 1.0000 | 4.5097 | 5.0345 |
| head_on | context_ext | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0003 | -0.0003 | 0.0250 | 1.0000 | 4.6823 | 4.8185 |
| head_on | context_ext | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.0409 | 0.0426 | 0.0000 | 1.0000 | 4.7300 | 4.6233 |
| head_on | context_ext | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.6309 | -0.2265 | 0.0917 | 1.0000 | 5.5452 | 14.6414 |
| head_on | context_ext | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.2846 | 0.0500 | 0.0000 | 1.0000 | 4.9605 | 4.6564 |
| crossing | context_ext | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0377 | -0.0377 | 0.0583 | 1.0000 | 5.1668 | 11.6878 |
| crossing | context_ext | EESM-MPC-ECBF | 0.0000 | 1.0000 | -0.0002 | -0.0050 | 0.0250 | 1.0000 | 5.4472 | 12.2840 |
| crossing | context_ext | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.2899 | -0.4999 | 0.3250 | 1.0000 | 7.3137 | 22.0822 |
| crossing | context_ext | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.0135 | 0.0363 | 0.0000 | 1.0000 | 6.6665 | 17.6694 |
| local_crowding | context_ext | Standard MPC-CBF | 0.0000 | 1.0000 | -0.0001 | -0.0001 | 0.0167 | 1.0000 | 4.5699 | 4.7280 |
| local_crowding | context_ext | EESM-MPC-ECBF | 1.0000 | 0.0000 | 0.1788 | 0.1814 | 0.0000 | 1.0000 | 4.5123 | 5.9083 |
| local_crowding | context_ext | SEESM w/o Feasibility-Preserving Update | 1.0000 | 0.0000 | 0.6402 | -0.2853 | 0.1000 | 1.0000 | 5.4685 | 10.1765 |
| local_crowding | context_ext | Proposed MPC-SECBF | 1.0000 | 0.0000 | 0.2816 | 0.0500 | 0.0000 | 1.0000 | 4.5093 | 4.4769 |
