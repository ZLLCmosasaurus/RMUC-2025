  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
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
      section.nData     = 89;
      section.data(89)  = dumData; %prealloc
      
	  ;% rtP.K
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.PIDController3_D
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 12;
	
	  ;% rtP.PIDController2_D
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 13;
	
	  ;% rtP.PIDController2_I
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 14;
	
	  ;% rtP.PIDController3_I
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 15;
	
	  ;% rtP.PIDController3_InitialConditionForFilter
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 16;
	
	  ;% rtP.PIDController2_InitialConditionForFilter
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 17;
	
	  ;% rtP.PIDController3_InitialConditionForIntegrator
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 18;
	
	  ;% rtP.PIDController2_InitialConditionForIntegrator
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 19;
	
	  ;% rtP.PIDController3_N
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 20;
	
	  ;% rtP.PIDController2_N
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 21;
	
	  ;% rtP.PIDController3_P
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 22;
	
	  ;% rtP.PIDController2_P
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 23;
	
	  ;% rtP.jump_control_jump_time
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 24;
	
	  ;% rtP.T1_Y0
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 25;
	
	  ;% rtP.T2_Y0
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 26;
	
	  ;% rtP.Gain26_Gain
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 27;
	
	  ;% rtP.Gain25_Gain
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 28;
	
	  ;% rtP.Gain33_Gain
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 29;
	
	  ;% rtP.Gain24_Gain
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 30;
	
	  ;% rtP.Gain40_Gain
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 31;
	
	  ;% rtP.Gain38_Gain
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 32;
	
	  ;% rtP.Gain22_Gain
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 33;
	
	  ;% rtP.Gain36_Gain
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 34;
	
	  ;% rtP.Integrator_IC
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 35;
	
	  ;% rtP.Step_Y0
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 36;
	
	  ;% rtP.Step_YFinal
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 37;
	
	  ;% rtP.Step16_Y0
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 38;
	
	  ;% rtP.Step16_YFinal
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 39;
	
	  ;% rtP.Step1_Y0
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 40;
	
	  ;% rtP.Step1_YFinal
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 41;
	
	  ;% rtP.Step13_Y0
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 42;
	
	  ;% rtP.Step13_YFinal
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 43;
	
	  ;% rtP.Step6_Y0
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 44;
	
	  ;% rtP.Step6_YFinal
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 45;
	
	  ;% rtP.Step14_Y0
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 46;
	
	  ;% rtP.Step14_YFinal
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 47;
	
	  ;% rtP.Step7_Y0
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 48;
	
	  ;% rtP.Step7_YFinal
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 49;
	
	  ;% rtP.Step12_Y0
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 50;
	
	  ;% rtP.Step12_YFinal
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 51;
	
	  ;% rtP.Step8_Y0
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 52;
	
	  ;% rtP.Step8_YFinal
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 53;
	
	  ;% rtP.Step15_Y0
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 54;
	
	  ;% rtP.Step15_YFinal
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 55;
	
	  ;% rtP.Step9_Y0
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 56;
	
	  ;% rtP.Step9_YFinal
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 57;
	
	  ;% rtP.Step11_Y0
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 58;
	
	  ;% rtP.Step11_YFinal
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 59;
	
	  ;% rtP.Saturation5_UpperSat
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 60;
	
	  ;% rtP.Saturation5_LowerSat
	  section.data(51).logicalSrcIdx = 50;
	  section.data(51).dtTransOffset = 61;
	
	  ;% rtP.Step4_Y0
	  section.data(52).logicalSrcIdx = 51;
	  section.data(52).dtTransOffset = 62;
	
	  ;% rtP.Step4_YFinal
	  section.data(53).logicalSrcIdx = 52;
	  section.data(53).dtTransOffset = 63;
	
	  ;% rtP.Step17_Y0
	  section.data(54).logicalSrcIdx = 53;
	  section.data(54).dtTransOffset = 64;
	
	  ;% rtP.Step17_YFinal
	  section.data(55).logicalSrcIdx = 54;
	  section.data(55).dtTransOffset = 65;
	
	  ;% rtP.Step5_Y0
	  section.data(56).logicalSrcIdx = 55;
	  section.data(56).dtTransOffset = 66;
	
	  ;% rtP.Step5_YFinal
	  section.data(57).logicalSrcIdx = 56;
	  section.data(57).dtTransOffset = 67;
	
	  ;% rtP.Step10_Y0
	  section.data(58).logicalSrcIdx = 57;
	  section.data(58).dtTransOffset = 68;
	
	  ;% rtP.Step10_YFinal
	  section.data(59).logicalSrcIdx = 58;
	  section.data(59).dtTransOffset = 69;
	
	  ;% rtP.Saturation1_UpperSat
	  section.data(60).logicalSrcIdx = 59;
	  section.data(60).dtTransOffset = 70;
	
	  ;% rtP.Saturation1_LowerSat
	  section.data(61).logicalSrcIdx = 60;
	  section.data(61).dtTransOffset = 71;
	
	  ;% rtP.Saturation2_UpperSat
	  section.data(62).logicalSrcIdx = 61;
	  section.data(62).dtTransOffset = 72;
	
	  ;% rtP.Saturation2_LowerSat
	  section.data(63).logicalSrcIdx = 62;
	  section.data(63).dtTransOffset = 73;
	
	  ;% rtP.Saturation3_UpperSat
	  section.data(64).logicalSrcIdx = 63;
	  section.data(64).dtTransOffset = 74;
	
	  ;% rtP.Saturation3_LowerSat
	  section.data(65).logicalSrcIdx = 64;
	  section.data(65).dtTransOffset = 75;
	
	  ;% rtP.Saturation4_UpperSat
	  section.data(66).logicalSrcIdx = 65;
	  section.data(66).dtTransOffset = 76;
	
	  ;% rtP.Saturation4_LowerSat
	  section.data(67).logicalSrcIdx = 66;
	  section.data(67).dtTransOffset = 77;
	
	  ;% rtP.Saturation6_UpperSat
	  section.data(68).logicalSrcIdx = 67;
	  section.data(68).dtTransOffset = 78;
	
	  ;% rtP.Saturation6_LowerSat
	  section.data(69).logicalSrcIdx = 68;
	  section.data(69).dtTransOffset = 79;
	
	  ;% rtP.Saturation7_UpperSat
	  section.data(70).logicalSrcIdx = 69;
	  section.data(70).dtTransOffset = 80;
	
	  ;% rtP.Saturation7_LowerSat
	  section.data(71).logicalSrcIdx = 70;
	  section.data(71).dtTransOffset = 81;
	
	  ;% rtP.Saturation8_UpperSat
	  section.data(72).logicalSrcIdx = 71;
	  section.data(72).dtTransOffset = 82;
	
	  ;% rtP.Saturation8_LowerSat
	  section.data(73).logicalSrcIdx = 72;
	  section.data(73).dtTransOffset = 83;
	
	  ;% rtP.Step2_Time
	  section.data(74).logicalSrcIdx = 73;
	  section.data(74).dtTransOffset = 84;
	
	  ;% rtP.Step2_Y0
	  section.data(75).logicalSrcIdx = 74;
	  section.data(75).dtTransOffset = 85;
	
	  ;% rtP.Step2_YFinal
	  section.data(76).logicalSrcIdx = 75;
	  section.data(76).dtTransOffset = 86;
	
	  ;% rtP.Step3_Time
	  section.data(77).logicalSrcIdx = 76;
	  section.data(77).dtTransOffset = 87;
	
	  ;% rtP.Step3_Y0
	  section.data(78).logicalSrcIdx = 77;
	  section.data(78).dtTransOffset = 88;
	
	  ;% rtP.Step3_YFinal
	  section.data(79).logicalSrcIdx = 78;
	  section.data(79).dtTransOffset = 89;
	
	  ;% rtP.Integrator4_IC
	  section.data(80).logicalSrcIdx = 79;
	  section.data(80).dtTransOffset = 90;
	
	  ;% rtP.Gain29_Gain
	  section.data(81).logicalSrcIdx = 80;
	  section.data(81).dtTransOffset = 91;
	
	  ;% rtP.Integrator2_IC
	  section.data(82).logicalSrcIdx = 81;
	  section.data(82).dtTransOffset = 92;
	
	  ;% rtP.Gain41_Gain
	  section.data(83).logicalSrcIdx = 82;
	  section.data(83).dtTransOffset = 93;
	
	  ;% rtP.Constant_Value
	  section.data(84).logicalSrcIdx = 83;
	  section.data(84).dtTransOffset = 94;
	
	  ;% rtP.Constant1_Value
	  section.data(85).logicalSrcIdx = 84;
	  section.data(85).dtTransOffset = 95;
	
	  ;% rtP.Constant10_Value
	  section.data(86).logicalSrcIdx = 85;
	  section.data(86).dtTransOffset = 96;
	
	  ;% rtP.Constant12_Value
	  section.data(87).logicalSrcIdx = 86;
	  section.data(87).dtTransOffset = 97;
	
	  ;% rtP.Value
	  section.data(88).logicalSrcIdx = 87;
	  section.data(88).dtTransOffset = 98;
	
	  ;% rtP._Value
	  section.data(89).logicalSrcIdx = 88;
	  section.data(89).dtTransOffset = 99;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
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
    nTotSects     = 1;
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
      section.nData     = 69;
      section.data(69)  = dumData; %prealloc
      
	  ;% rtB.eh2cmnfpcd
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.ne53mrvzfq
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 32;
	
	  ;% rtB.nwuie3h1zj
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 45;
	
	  ;% rtB.liiv3zehzs
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 46;
	
	  ;% rtB.hkrrrfopbq
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 47;
	
	  ;% rtB.oaaq01l5fu
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 48;
	
	  ;% rtB.oqhhk34uis
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 49;
	
	  ;% rtB.fx0alrhetp
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 50;
	
	  ;% rtB.c4gioprvct
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 51;
	
	  ;% rtB.ewa15giale
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 52;
	
	  ;% rtB.cfmoived0b
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 53;
	
	  ;% rtB.jsqo11gz4n
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 54;
	
	  ;% rtB.fvygvwpi1d
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 55;
	
	  ;% rtB.nkvqeuwnam
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 56;
	
	  ;% rtB.cufla4blkp
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 57;
	
	  ;% rtB.a5rxjeqmnm
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 58;
	
	  ;% rtB.lp2kjlaox2
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 59;
	
	  ;% rtB.d4fftorabg
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 60;
	
	  ;% rtB.o0acnzn2gn
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 61;
	
	  ;% rtB.lno5gwkvsm
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 62;
	
	  ;% rtB.m0bxxy51bt
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 68;
	
	  ;% rtB.ct5ptiiqfm
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 70;
	
	  ;% rtB.egyva5trzl
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 71;
	
	  ;% rtB.hhykaoe4za
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 72;
	
	  ;% rtB.emby3igyk1
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 73;
	
	  ;% rtB.lunhybn4j1
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 74;
	
	  ;% rtB.bibxaqp42j
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 75;
	
	  ;% rtB.ngl2rc54wx
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 76;
	
	  ;% rtB.mkoprhigc1
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 77;
	
	  ;% rtB.p4cj0opfml
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 78;
	
	  ;% rtB.kxvii2moqu
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 79;
	
	  ;% rtB.dz5u3bubv3
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 80;
	
	  ;% rtB.b0mlwzzkzn
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 81;
	
	  ;% rtB.bpbscqkzum
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 82;
	
	  ;% rtB.lno5gwkvsmk
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 83;
	
	  ;% rtB.kourgu0gp0
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 89;
	
	  ;% rtB.luhg3pfwco
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 91;
	
	  ;% rtB.fasi4utync
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 92;
	
	  ;% rtB.mj2xnu20kl
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 93;
	
	  ;% rtB.cp1jjx2a4p
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 94;
	
	  ;% rtB.furrbhkomt
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 95;
	
	  ;% rtB.mrjzncw5q4
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 96;
	
	  ;% rtB.imk0sd2jrm
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 97;
	
	  ;% rtB.jteoenmoec
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 98;
	
	  ;% rtB.jw4ohhvqrt
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 99;
	
	  ;% rtB.f1443sccd2
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 100;
	
	  ;% rtB.jfiugehczg
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 101;
	
	  ;% rtB.hdeqxnl10m
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 102;
	
	  ;% rtB.djd0cbchne
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 103;
	
	  ;% rtB.hl2vsnjvah
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 104;
	
	  ;% rtB.hfyfla4srx
	  section.data(51).logicalSrcIdx = 50;
	  section.data(51).dtTransOffset = 105;
	
	  ;% rtB.nlnzoie4ft
	  section.data(52).logicalSrcIdx = 51;
	  section.data(52).dtTransOffset = 106;
	
	  ;% rtB.affemz5v0v
	  section.data(53).logicalSrcIdx = 52;
	  section.data(53).dtTransOffset = 107;
	
	  ;% rtB.amehlztnnv
	  section.data(54).logicalSrcIdx = 53;
	  section.data(54).dtTransOffset = 108;
	
	  ;% rtB.fviua014cu
	  section.data(55).logicalSrcIdx = 54;
	  section.data(55).dtTransOffset = 109;
	
	  ;% rtB.kaed4psp05
	  section.data(56).logicalSrcIdx = 55;
	  section.data(56).dtTransOffset = 110;
	
	  ;% rtB.chcwz4ujvj
	  section.data(57).logicalSrcIdx = 56;
	  section.data(57).dtTransOffset = 111;
	
	  ;% rtB.nxxivmzdo0
	  section.data(58).logicalSrcIdx = 57;
	  section.data(58).dtTransOffset = 112;
	
	  ;% rtB.k3exkay1ay
	  section.data(59).logicalSrcIdx = 58;
	  section.data(59).dtTransOffset = 113;
	
	  ;% rtB.kma1dpwhhf
	  section.data(60).logicalSrcIdx = 59;
	  section.data(60).dtTransOffset = 114;
	
	  ;% rtB.nik4ggyist
	  section.data(61).logicalSrcIdx = 60;
	  section.data(61).dtTransOffset = 115;
	
	  ;% rtB.ludhxwcsap
	  section.data(62).logicalSrcIdx = 61;
	  section.data(62).dtTransOffset = 119;
	
	  ;% rtB.jl2dsktpay
	  section.data(63).logicalSrcIdx = 62;
	  section.data(63).dtTransOffset = 123;
	
	  ;% rtB.clrzjaih1v
	  section.data(64).logicalSrcIdx = 63;
	  section.data(64).dtTransOffset = 127;
	
	  ;% rtB.dmfybrfnod
	  section.data(65).logicalSrcIdx = 64;
	  section.data(65).dtTransOffset = 131;
	
	  ;% rtB.l1345vcjcj
	  section.data(66).logicalSrcIdx = 65;
	  section.data(66).dtTransOffset = 135;
	
	  ;% rtB.bdqfv1d2cw
	  section.data(67).logicalSrcIdx = 66;
	  section.data(67).dtTransOffset = 139;
	
	  ;% rtB.lbhtwulbcm
	  section.data(68).logicalSrcIdx = 67;
	  section.data(68).dtTransOffset = 140;
	
	  ;% rtB.p1iy05u0ee
	  section.data(69).logicalSrcIdx = 68;
	  section.data(69).dtTransOffset = 141;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
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
    nTotSects     = 5;
    sectIdxOffset = 1;
    
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
      section.nData     = 9;
      section.data(9)  = dumData; %prealloc
      
	  ;% rtDW.pvnlmwwllm
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.mxlt45ij3o
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtDW.f1r4g4whvp
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 4;
	
	  ;% rtDW.aokoeghayj
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 6;
	
	  ;% rtDW.kulfjc4a2j
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 8;
	
	  ;% rtDW.leayjggnf1
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 10;
	
	  ;% rtDW.ooldbvhge3
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 12;
	
	  ;% rtDW.cq3o2ec5fw
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 13;
	
	  ;% rtDW.h5eppyllsj
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 14;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 32;
      section.data(32)  = dumData; %prealloc
      
	  ;% rtDW.eukrvu0tgn
	  section.data(1).logicalSrcIdx = 9;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.gahukgbj3h
	  section.data(2).logicalSrcIdx = 10;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.bq244s1auv
	  section.data(3).logicalSrcIdx = 11;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.jsgic4uyjm
	  section.data(4).logicalSrcIdx = 12;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.cxtggfsfxm
	  section.data(5).logicalSrcIdx = 13;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.go2zdudlka
	  section.data(6).logicalSrcIdx = 14;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.jg2ci40rqj
	  section.data(7).logicalSrcIdx = 15;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.bmoh4rr253
	  section.data(8).logicalSrcIdx = 16;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.h2fxcrou4b
	  section.data(9).logicalSrcIdx = 17;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.gmcd5pdmpl
	  section.data(10).logicalSrcIdx = 18;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.pxbtt2xwqv.LoggedData
	  section.data(11).logicalSrcIdx = 19;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.cgimjkdh23.LoggedData
	  section.data(12).logicalSrcIdx = 20;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.hpnbv4vlgo.LoggedData
	  section.data(13).logicalSrcIdx = 21;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.ditp2lm0v5.LoggedData
	  section.data(14).logicalSrcIdx = 22;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.heiomuhezr.LoggedData
	  section.data(15).logicalSrcIdx = 23;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.euizrqhecv.LoggedData
	  section.data(16).logicalSrcIdx = 24;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.ghm5gdrsfi.LoggedData
	  section.data(17).logicalSrcIdx = 25;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtDW.mxi5rvja4b.LoggedData
	  section.data(18).logicalSrcIdx = 26;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtDW.evkpec522i.LoggedData
	  section.data(19).logicalSrcIdx = 27;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtDW.iwbd2ehmq0.LoggedData
	  section.data(20).logicalSrcIdx = 28;
	  section.data(20).dtTransOffset = 19;
	
	  ;% rtDW.nrwpiptbpa.LoggedData
	  section.data(21).logicalSrcIdx = 29;
	  section.data(21).dtTransOffset = 20;
	
	  ;% rtDW.ae5ijrvxng.LoggedData
	  section.data(22).logicalSrcIdx = 30;
	  section.data(22).dtTransOffset = 21;
	
	  ;% rtDW.ijaanpy3ox.LoggedData
	  section.data(23).logicalSrcIdx = 31;
	  section.data(23).dtTransOffset = 22;
	
	  ;% rtDW.oj0cyu2zrs.LoggedData
	  section.data(24).logicalSrcIdx = 32;
	  section.data(24).dtTransOffset = 23;
	
	  ;% rtDW.nebmwciflv.LoggedData
	  section.data(25).logicalSrcIdx = 33;
	  section.data(25).dtTransOffset = 24;
	
	  ;% rtDW.nxzkrnlszw.LoggedData
	  section.data(26).logicalSrcIdx = 34;
	  section.data(26).dtTransOffset = 25;
	
	  ;% rtDW.nssaobtpxs.LoggedData
	  section.data(27).logicalSrcIdx = 35;
	  section.data(27).dtTransOffset = 26;
	
	  ;% rtDW.loahdpiobx
	  section.data(28).logicalSrcIdx = 36;
	  section.data(28).dtTransOffset = 27;
	
	  ;% rtDW.hpnunkuqpe
	  section.data(29).logicalSrcIdx = 37;
	  section.data(29).dtTransOffset = 28;
	
	  ;% rtDW.ojbc53o3xt
	  section.data(30).logicalSrcIdx = 38;
	  section.data(30).dtTransOffset = 29;
	
	  ;% rtDW.bzw2pamqyl
	  section.data(31).logicalSrcIdx = 39;
	  section.data(31).dtTransOffset = 30;
	
	  ;% rtDW.pifpfpnwqn
	  section.data(32).logicalSrcIdx = 40;
	  section.data(32).dtTransOffset = 31;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 27;
      section.data(27)  = dumData; %prealloc
      
	  ;% rtDW.nzztmagq15
	  section.data(1).logicalSrcIdx = 41;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.d5bjwf4dqd
	  section.data(2).logicalSrcIdx = 42;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.g24kz15cah
	  section.data(3).logicalSrcIdx = 43;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.kbppfzielz
	  section.data(4).logicalSrcIdx = 44;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.acajdtsjkp
	  section.data(5).logicalSrcIdx = 45;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.pkcg1jz5ka
	  section.data(6).logicalSrcIdx = 46;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.ei5ynuzssw
	  section.data(7).logicalSrcIdx = 47;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.ekmth0nsne
	  section.data(8).logicalSrcIdx = 48;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.b15ck32r2h
	  section.data(9).logicalSrcIdx = 49;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.gy03yk4tjx
	  section.data(10).logicalSrcIdx = 50;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.dgksrckq3m
	  section.data(11).logicalSrcIdx = 51;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.cojkg3rcgw
	  section.data(12).logicalSrcIdx = 52;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.a04qfypmbt
	  section.data(13).logicalSrcIdx = 53;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.hjkpjosmn0
	  section.data(14).logicalSrcIdx = 54;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.huukfn1whc
	  section.data(15).logicalSrcIdx = 55;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.bj5n1250ac
	  section.data(16).logicalSrcIdx = 56;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.g1jau2oi4y
	  section.data(17).logicalSrcIdx = 57;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtDW.phsfelswbk
	  section.data(18).logicalSrcIdx = 58;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtDW.emdjtjjfhz
	  section.data(19).logicalSrcIdx = 59;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtDW.i1mxirq5kb
	  section.data(20).logicalSrcIdx = 60;
	  section.data(20).dtTransOffset = 19;
	
	  ;% rtDW.dhbkidd3zo
	  section.data(21).logicalSrcIdx = 61;
	  section.data(21).dtTransOffset = 20;
	
	  ;% rtDW.iy1pcv3xo4
	  section.data(22).logicalSrcIdx = 62;
	  section.data(22).dtTransOffset = 21;
	
	  ;% rtDW.eqwdrnbx1x
	  section.data(23).logicalSrcIdx = 63;
	  section.data(23).dtTransOffset = 22;
	
	  ;% rtDW.pny00ejsqz
	  section.data(24).logicalSrcIdx = 64;
	  section.data(24).dtTransOffset = 23;
	
	  ;% rtDW.aqiduhop2a
	  section.data(25).logicalSrcIdx = 65;
	  section.data(25).dtTransOffset = 24;
	
	  ;% rtDW.fpt2stpuqn
	  section.data(26).logicalSrcIdx = 66;
	  section.data(26).dtTransOffset = 25;
	
	  ;% rtDW.ctsfqdbiky
	  section.data(27).logicalSrcIdx = 67;
	  section.data(27).dtTransOffset = 26;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.kpjnyoicn4
	  section.data(1).logicalSrcIdx = 68;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtDW.mif2ebuzod
	  section.data(1).logicalSrcIdx = 69;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.o50jinjfvw
	  section.data(2).logicalSrcIdx = 70;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.kikuoijf40
	  section.data(3).logicalSrcIdx = 71;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
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


  targMap.checksum0 = 3625382941;
  targMap.checksum1 = 3387521915;
  targMap.checksum2 = 3494444731;
  targMap.checksum3 = 3437323851;

