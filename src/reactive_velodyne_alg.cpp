#include "reactive_velodyne_alg.h"

ReactiveVelodyneAlgorithm::ReactiveVelodyneAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

ReactiveVelodyneAlgorithm::~ReactiveVelodyneAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void ReactiveVelodyneAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// ReactiveVelodyneAlgorithm Public API
