#ifndef COOLDOWN_HPP
#define COOLDOWN_HPP

class Cooldown {
public:
	float cooldownTime;
	
	float startTime;
	float endTime;
public:
	Cooldown(float cooldownTime);

	void start();
	bool inCooldown();
};

#endif