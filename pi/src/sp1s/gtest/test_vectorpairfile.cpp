#include <gtest/gtest.h>
#include "../src/util/vectorpairfile.h"

void InitVectorPairInstance(vectorPair * pInstance)
{
	pInstance->Reset(10);
	for(int i = 0; i< 10; i++)
	{
		pInstance->X[i] = i+1;
		pInstance->Y[i] = (i+1) * 10.0;
	}
	pInstance->compute();
}

TEST(vectorPair, Offset_Factor_Is_Right){
	vectorPair instance;
	InitVectorPairInstance(&instance);
	ASSERT_DOUBLE_EQ(10.0, instance.m_dFactor);
	ASSERT_DOUBLE_EQ(0.0, instance.m_dOffset);
}

TEST(vectorPair, Follow_First_Two_Samples_When_Target_Too_Samll){
	vectorPair instance;
	InitVectorPairInstance(&instance);

	// Change the sample of the first two samples
	instance.X[0] = 1;
	instance.Y[0] = 50;
	instance.X[1] = 2;
	instance.Y[1] = 100;
	double factor = 50.0 / 1.0;
	ASSERT_DOUBLE_EQ(factor,(instance.Y[1] - instance.Y[0])/(instance.X[1] - instance.X[0]));

	double target_y = 5.0;
	double target_x = instance.InterpolateY2X(target_y);
	double expected_x = target_y / factor;
	ASSERT_DOUBLE_EQ(expected_x, target_x);
}

TEST(vectorPair, Follow_Last_Two_Samples_When_Target_Too_Big){
	vectorPair instance;
	InitVectorPairInstance(&instance);

	// Change the sample of the last two samples
	instance.X[8] = 9;
	instance.Y[8] = 90;
	instance.X[9] = 10;
	instance.Y[9] = 190;
	double factor = (instance.Y[9] - instance.Y[8])/(instance.X[9] - instance.X[8]);

	double target_y = 200.0;
	double target_x = instance.InterpolateY2X(target_y);
	double expected_x = instance.X[9] + (target_y - instance.Y[9]) / factor;
	ASSERT_DOUBLE_EQ(expected_x, target_x);
}

TEST(vectorPair, Use_SampleValue_When_Target_At_Sample_Exactly)
{
	vectorPair instance;
	InitVectorPairInstance(&instance);

	double target_y = instance.Y[0];
	double expected_x = 0.1;
	instance.X[0] = expected_x;
	double target_x = instance.InterpolateY2X(target_y);
	ASSERT_DOUBLE_EQ(expected_x, target_x);

	target_y = instance.Y[5];
    expected_x = 6.1;
	instance.X[5] = expected_x;
	target_x = instance.InterpolateY2X(target_y);
	ASSERT_DOUBLE_EQ(expected_x, target_x);

	target_y = instance.Y[9];
	expected_x = 9.1;
	instance.X[9] = expected_x;
	target_x = instance.InterpolateY2X(target_y);
	ASSERT_DOUBLE_EQ(expected_x, target_x);
}

TEST(vectorPair, Interpolate_X_When_Target_InBetween_TwoSamples)
{
	vectorPair instance;
	InitVectorPairInstance(&instance);
	instance.Y[5] = 100;
	instance.Y[6] = 200;
	double factor = (instance.Y[6]-instance.Y[5]) / (instance.X[6]-instance.X[5]);
	double target_y = 120;
	double expected_x = instance.X[5] + (target_y - instance.Y[5])/factor;
	double target_x = instance.InterpolateY2X(target_y);
	ASSERT_DOUBLE_EQ(expected_x, target_x);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
