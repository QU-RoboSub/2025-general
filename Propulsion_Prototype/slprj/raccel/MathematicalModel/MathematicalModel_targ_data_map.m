    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 2;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 51;
            section.data(51)  = dumData; %prealloc

                    ;% rtP.V
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.g
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.m
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.rho
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtP.DepthControl_D
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtP.PIDController2_D
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtP.DepthControl_I
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtP.PIDController2_I
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtP.DepthControl_InitialConditionForFilter
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtP.PIDController2_InitialConditionForFilter
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtP.DepthControl_InitialConditionForIntegrator
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtP.PIDController2_InitialConditionForIntegrator
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtP.DepthControl_N
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% rtP.PIDController2_N
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% rtP.DepthControl_P
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% rtP.MATLABSystem_SampleTime
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% rtP.DesiredDepthmm_Time
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% rtP.DesiredDepthmm_Y0
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 17;

                    ;% rtP.DesiredDepthmm_YFinal
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 18;

                    ;% rtP.StateSpace_A_pr
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 19;

                    ;% rtP.StateSpace_B_pr
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20;

                    ;% rtP.StateSpace_C_pr
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 21;

                    ;% rtP.StateSpace_InitialCondition
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 22;

                    ;% rtP.Integrator_gainval
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 23;

                    ;% rtP.Filter_gainval
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 24;

                    ;% rtP.ThrustControltoPWM_tableData
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 25;

                    ;% rtP.ThrustControltoPWM_bp01Data
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 226;

                    ;% rtP.T200PowerCurve_tableData
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 427;

                    ;% rtP.T200PowerCurve_bp01Data
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 628;

                    ;% rtP.Disturbance_Amp
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 829;

                    ;% rtP.Disturbance_Bias
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 830;

                    ;% rtP.Disturbance_Freq
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 831;

                    ;% rtP.Disturbance_Phase
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 832;

                    ;% rtP.Gain1_Gain
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 833;

                    ;% rtP.StateSpace_A_pr_jy4jjiufdj
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 834;

                    ;% rtP.StateSpace_B_pr_dn004bbuke
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 836;

                    ;% rtP.StateSpace_C_pr_jgvflc2hx0
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 837;

                    ;% rtP.StateSpace_InitialCondition_cvrppagalz
                    section.data(38).logicalSrcIdx = 37;
                    section.data(38).dtTransOffset = 838;

                    ;% rtP.mtomm_Gain
                    section.data(39).logicalSrcIdx = 38;
                    section.data(39).dtTransOffset = 840;

                    ;% rtP.DepthLimit_UpperSat
                    section.data(40).logicalSrcIdx = 39;
                    section.data(40).dtTransOffset = 841;

                    ;% rtP.DepthLimit_LowerSat
                    section.data(41).logicalSrcIdx = 40;
                    section.data(41).dtTransOffset = 842;

                    ;% rtP.SensorNoise_Minimum
                    section.data(42).logicalSrcIdx = 41;
                    section.data(42).dtTransOffset = 843;

                    ;% rtP.SensorNoise_Maximum
                    section.data(43).logicalSrcIdx = 42;
                    section.data(43).dtTransOffset = 844;

                    ;% rtP.SensorNoise_Seed
                    section.data(44).logicalSrcIdx = 43;
                    section.data(44).dtTransOffset = 845;

                    ;% rtP.Constant1_Value
                    section.data(45).logicalSrcIdx = 44;
                    section.data(45).dtTransOffset = 846;

                    ;% rtP.uDLookupTable1_tableData
                    section.data(46).logicalSrcIdx = 45;
                    section.data(46).dtTransOffset = 847;

                    ;% rtP.uDLookupTable1_bp01Data
                    section.data(47).logicalSrcIdx = 46;
                    section.data(47).dtTransOffset = 1648;

                    ;% rtP.uDLookupTable2_tableData
                    section.data(48).logicalSrcIdx = 47;
                    section.data(48).dtTransOffset = 2449;

                    ;% rtP.uDLookupTable2_bp01Data
                    section.data(49).logicalSrcIdx = 48;
                    section.data(49).dtTransOffset = 3250;

                    ;% rtP.SpeedMultiplier_Value
                    section.data(50).logicalSrcIdx = 49;
                    section.data(50).dtTransOffset = 4051;

                    ;% rtP.Gain_Gain
                    section.data(51).logicalSrcIdx = 50;
                    section.data(51).dtTransOffset = 4052;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 12;
            section.data(12)  = dumData; %prealloc

                    ;% rtP.StateSpace_A_ir
                    section.data(1).logicalSrcIdx = 51;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.StateSpace_A_jc
                    section.data(2).logicalSrcIdx = 52;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.StateSpace_B_ir
                    section.data(3).logicalSrcIdx = 53;
                    section.data(3).dtTransOffset = 3;

                    ;% rtP.StateSpace_B_jc
                    section.data(4).logicalSrcIdx = 54;
                    section.data(4).dtTransOffset = 4;

                    ;% rtP.StateSpace_C_ir
                    section.data(5).logicalSrcIdx = 55;
                    section.data(5).dtTransOffset = 6;

                    ;% rtP.StateSpace_C_jc
                    section.data(6).logicalSrcIdx = 56;
                    section.data(6).dtTransOffset = 7;

                    ;% rtP.StateSpace_A_ir_d34gaajqxg
                    section.data(7).logicalSrcIdx = 57;
                    section.data(7).dtTransOffset = 9;

                    ;% rtP.StateSpace_A_jc_ogjw4zhixw
                    section.data(8).logicalSrcIdx = 58;
                    section.data(8).dtTransOffset = 11;

                    ;% rtP.StateSpace_B_ir_o1eztmk1co
                    section.data(9).logicalSrcIdx = 59;
                    section.data(9).dtTransOffset = 14;

                    ;% rtP.StateSpace_B_jc_k02fgthtcj
                    section.data(10).logicalSrcIdx = 60;
                    section.data(10).dtTransOffset = 15;

                    ;% rtP.StateSpace_C_ir_n00n3nmwj5
                    section.data(11).logicalSrcIdx = 61;
                    section.data(11).dtTransOffset = 17;

                    ;% rtP.StateSpace_C_jc_jwkolzzbma
                    section.data(12).logicalSrcIdx = 62;
                    section.data(12).dtTransOffset = 18;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 3;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 22;
            section.data(22)  = dumData; %prealloc

                    ;% rtB.lt1rdvoctr
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.dttgtgicym
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtB.koclthgufh
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtB.ot4biexhrc
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtB.os02ukstpo
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtB.mqu2twqjlp
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtB.l5manf4ikg
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtB.lny4bmp0zj
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtB.acg1d02c2x
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtB.j4ai4z50gk
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtB.ks2yz4xb4p
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtB.kjm434vqfg
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtB.od11d41rmb
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% rtB.kta3pakdpy
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% rtB.l3gxjc55ml
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% rtB.jvycygbmvu
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% rtB.isocrfb3eq
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% rtB.hiu3ufhlyn
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 17;

                    ;% rtB.b0vho2tiwj
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 18;

                    ;% rtB.oihzkslm5h
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 19;

                    ;% rtB.kdvlwjj4tq
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20;

                    ;% rtB.j1h0no0d4b
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 21;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtB.dzv1mxhlqy
                    section.data(1).logicalSrcIdx = 22;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtB.ajoc0paeou
                    section.data(1).logicalSrcIdx = 23;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(3) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 7;
        sectIdxOffset = 3;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.kyf2nclza1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.ojmumzzinc
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% rtDW.gdqbf2crkk
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.d2gjyneiwi
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.mqqpv5nckr
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 7;
            section.data(7)  = dumData; %prealloc

                    ;% rtDW.ayjhvvhiab.AQHandles
                    section.data(1).logicalSrcIdx = 5;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.gkjdrzz2x1.LoggedData
                    section.data(2).logicalSrcIdx = 6;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.bindmu0m5v.LoggedData
                    section.data(3).logicalSrcIdx = 7;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.ag142pt11j.LoggedData
                    section.data(4).logicalSrcIdx = 8;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.pjjo4vsiiw.LoggedData
                    section.data(5).logicalSrcIdx = 9;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.k53jbvhb4r.LoggedData
                    section.data(6).logicalSrcIdx = 10;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.p00iq10u1v.LoggedData
                    section.data(7).logicalSrcIdx = 11;
                    section.data(7).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.lzqjq4aymt
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.e3ooy4be24
                    section.data(1).logicalSrcIdx = 13;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.h20vrohnnu
                    section.data(2).logicalSrcIdx = 14;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.cr445o1xte
                    section.data(1).logicalSrcIdx = 15;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.if4q3i004h
                    section.data(2).logicalSrcIdx = 16;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 109747031;
    targMap.checksum1 = 2709809023;
    targMap.checksum2 = 2004263737;
    targMap.checksum3 = 379092649;

