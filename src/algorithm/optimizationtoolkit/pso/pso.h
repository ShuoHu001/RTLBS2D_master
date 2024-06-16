#ifndef RTLBS_PSO
#define RTLBS_PSO

#include "rtlbs.h"
#include "utility/define.h"

template<typename T>
class ParticleSwarmOptimization {
private:
	int m_numParticles;
	int m_dimensions;
	int m_maxIterations;
	double m_inertiaWeight;
	double m_cognitiveWeight;
	double m_socialWeight;
	std::vector<T> m_particles;
	T m_globalBest;

public:
	ParticleSwarmOptimization(int numParticles, int dimensions, int maxIterations, double inertiaWeight, double cognitiveWeight, double socialWeight) 
		: m_numParticles(numParticles)
		, m_dimensions(dimensions)
		, m_maxIterations(maxIterations)
		, m_inertiaWeight(inertiaWeight)
		, m_cognitiveWeight(cognitiveWeight)
		, m_socialWeight(socialWeight)
	{
	}

	T Run() {													//��������Ⱥ�㷨
		//��ʼ������Ⱥ
		for (int i = 0; i < m_numParticles; ++i) {
			T particle(m_dimensions);
			m_particles.push_back(particle);
		}

		m_globalBest.m_bestFitness = std::numeric_limits<double>::max();		//��ʼ��ȫ��λ�ü�����Ӧ��

		//������������Ⱥ
		for (int iter = 0; iter < m_maxIterations; ++iter) {
			updateParticles();													//�������ӵ�λ�ú��ٶ�

			for (auto& particle : m_particles) {
				if (particle.m_bestFitness < m_globalBest.m_bestFitness) {
					m_globalBest = particle;
				}
			}
		}

		return m_globalBest;
	}

private:
	void updateParticles() {									//�������ӵ�λ�ú��ٶ�
		for (auto& particle : m_particles) {
			for (int i = 0; i < m_dimensions; ++i) {			//���������ٶ�
				double r1 = ((double)rand()) / RAND_MAX;
				double r2 = ((double)rand()) / RAND_MAX;
				double cognitiveComponent = m_cognitiveWeight * r1 * *(particle.m_bestPosition[i] - particle.m_position[i]);
				double socialComponent = m_socialWeight * r2 * (m_globalBest.m_position[i] - particle.m_position[i]);
				particle.m_velocity[i] = m_inertiaWeight * particle.m_velocity[i] + cognitiveComponent + socialComponent;
			}

			for (int i = 0; i < m_dimensions; ++i) {			//��������λ��
				particle.m_position[i] += particle.m_velocity[i];
			}

			double particleFitness = particle.CalFitness();		//����������Ӧ��
			if (particleFitness < particle.m_bestFitness) {
				particle.m_bestFitness = particleFitness;
				particle.m_bestPosition = particle.m_position
			}
		}
	}
};

#endif
