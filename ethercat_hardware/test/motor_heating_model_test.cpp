#include <gtest/gtest.h>
#include <string.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "ethercat_hardware/motor_heating_model.h"


using ethercat_hardware::MotorHeatingModel;
using ethercat_hardware::MotorHeatingModelParameters;
using ethercat_hardware::MotorHeatingModelParametersEepromConfig;
using ethercat_hardware::MotorHeatingModelCommon;



/**
 * Helper fucntion, generate motor heating parameters that are similar to 
 * caster rotation motor parameters (as defined in actuators.conf)
 */
MotorHeatingModelParameters casterMotorHeatingModelParams()
{
  MotorHeatingModelParameters params;
  params.housing_to_ambient_thermal_resistance_ = 4.65;
  params.winding_to_housing_thermal_resistance_ = 1.93;
  params.winding_thermal_time_constant_         = 41.6;
  params.housing_thermal_time_constant_         = 1120.0;
  params.max_winding_temperature_               = 155.0;
  return params;
}



class UpdateFromDowntimeTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // Motor params (for caster motor)
    MotorHeatingModelParameters params( casterMotorHeatingModelParams() );
    
    boost::shared_ptr<MotorHeatingModelCommon> common = boost::make_shared<MotorHeatingModelCommon>();
    common->load_save_files_   = false;
    common->update_save_files_ = false;
    common->save_directory_    = "/tmp/motor_heating_model";
    
    // Make two motor heating model objects
    model_1_ = boost::make_shared<MotorHeatingModel>(params, "motor_1", "hwid", "actuator_name");
    model_2_ = boost::make_shared<MotorHeatingModel>(params, "motor_2", "hwid", "actuator_name");
  }


public:

  void runUpdate(int cycles, double amblient_temperature)
  {
    // Simulate each for model for same ammount of time
    double interval = 0.001; // 1ms
    double downtime = cycles * interval;
    double ambient_temperature = 30.0;

    // Sim first model with small timesteps
    static const double heating_power = 0.0;
    for (int i=0; i<cycles; ++i)
    {
      model_1_->update(heating_power, ambient_temperature, interval);
    }
    
    // Sim second with update function
    model_2_->updateFromDowntime(downtime, ambient_temperature);
  }

  boost::shared_ptr<MotorHeatingModel> model_1_;
  boost::shared_ptr<MotorHeatingModel> model_2_;
};


TEST_F(UpdateFromDowntimeTest, DowntimeUpdate10Seconds)
{
  double ambient_temperature = 30.0;
  
  // Compare resulting temperatures (assert temperatures are within 0.1 degrees Celcius

  int cycles = 1000;

  for (int i=0; i<10; ++i)
  {
    runUpdate(cycles, ambient_temperature);
    EXPECT_TRUE(  std::abs(  model_1_->getWindingTemperature() - model_2_->getWindingTemperature() ) < 0.1 );
    EXPECT_TRUE(  std::abs(  model_1_->getHousingTemperature() - model_2_->getHousingTemperature() ) < 0.1 );
  }
}


TEST_F(UpdateFromDowntimeTest, DowntimeUpdate100Seconds)
{
  int cycles = 100 * 1000;
  double ambient_temperature = 30.0;
  runUpdate(cycles, ambient_temperature);
  
  // Compare resulting temperatures (assert temperatures are within 0.1 degrees Celcius
  EXPECT_TRUE(  std::abs(  model_1_->getWindingTemperature() - model_2_->getWindingTemperature() ) < 0.1 );
  EXPECT_TRUE(  std::abs(  model_1_->getHousingTemperature() - model_2_->getHousingTemperature() ) < 0.1 );
}


TEST_F(UpdateFromDowntimeTest, DowntimeUpdate1000Seconds)
{
  int cycles = 1000 * 1000;
  double ambient_temperature = 30.0;
  runUpdate(cycles, ambient_temperature);
  
  // Compare resulting temperatures (assert temperatures are within 0.1 degrees Celcius
  EXPECT_TRUE(  std::abs(  model_1_->getWindingTemperature() - model_2_->getWindingTemperature() ) < 0.1 );
  EXPECT_TRUE(  std::abs(  model_1_->getHousingTemperature() - model_2_->getHousingTemperature() ) < 0.1 );
}


TEST_F(UpdateFromDowntimeTest, DowntimeUpdate10000Seconds)
{
  int cycles = 10000 * 1000;
  double ambient_temperature = 30.0;
  runUpdate(cycles, ambient_temperature);
  
  // Compare resulting temperatures (assert temperatures are within 0.1 degrees Celcius
  EXPECT_TRUE(  std::abs(  model_1_->getWindingTemperature() - model_2_->getWindingTemperature() ) < 0.1 );
  EXPECT_TRUE(  std::abs(  model_1_->getHousingTemperature() - model_2_->getHousingTemperature() ) < 0.1 );
}



TEST(MotorHeatingModelParametersEepromConfig, SelfConsistantCRC)
{
  MotorHeatingModelParametersEepromConfig config;
  memset(&config, 0, sizeof(config));
  
  config.generateCRC();
  
  EXPECT_TRUE( config.verifyCRC() );  
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
