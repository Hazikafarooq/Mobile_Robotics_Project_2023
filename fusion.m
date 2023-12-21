function [initialAtt,initialPos,initialVel,GyroBias,AccelBias] = fusion(initialAtt,initialPos,initialVel,GyroBias,AccelBias)

    gndFusion.State(1:4) = compact(initialAtt).';
    gndFusion.State(5:7) = GyroBias;
    gndFusion.State(8:10) = initialPos.';
    gndFusion.State(11:13) = initialVel.';
    gndFusion.State(14:16) = AccelBias;