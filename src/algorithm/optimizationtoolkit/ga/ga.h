#ifndef RTLBS_GA
#define RTLBS_GA

#include "rtlbs.h"
#include "utility/define.h"

template<typename T>
class GeneticAlgorithm {
private:
	int m_populationSize;							/** @brief	��Ⱥ����	*/
	int m_chromosomeLength;							/** @brief	Ⱦɫ�峤��	*/
	double m_crossoverRate;							/** @brief	������	*/
	double m_mutationRate;							/** @brief	������	*/
	int m_generations;								/** @brief	��������	*/

	std::vector<T> population;

	double (*fitnessFunction)(T);					//��Ӧ�Ⱥ���ָ��

	void (* selectFunction)(std::vector<T>&);		//ѡ���������ָ��

	void (*crossoverFunction)(T&, T&);				//�����������ָ��

	void (*mutationFunction)(T&);					//�����������ָ��

	


private:
	void initializePopulation() {					//��ʼ����Ⱥ
		population.clear();
		for (int i = 0; i < m_populationSize; ++i) {
			T individual;
			population.push_back(individual);
		}
	}

	void calculateFitness() {						//������Ⱥ��Ӧ��
		for (int i = 0; i < m_populationSize; ++i) {
			population[i].fitness = fitnessFunction(population[i]);
		}
	}

	void crossover() {								//���򽻲����
		for (int i = 0; i < m_populationSize; ++i) {
			T& mateFather = population[i];
			int mateId = rand();
			if (mateId < m_crossoverRate * RAND_MAX) {
				int j = mateId % m_populationSize;
				T& mateMother = population[j];
				crossoverFunction(mateFather, mateMother);
			}
		}
	}

	void mutate() {									//����������
		for (int i = 0; i < m_populationSize; ++i) {
			if (rand() < m_mutationRate * RAND_MAX) {
				T& individual = population[i];
				mutationFunction(individual);
			}
		}
	}

public:
	GeneticAlgorithm(int populationSize, int chromosomeLength, double crossoverRate, double mutationRate, int generations) {
		fitnessFunction = nullptr;
		selectFunction = nullptr;
		crossoverFunction = nullptr;
		mutationFunction = nullptr;
	}

	void SetFitnessFunction(double (*fitnessFunctionPtr)(T)) {
		fitnessFunction = fitnessFunctionPtr;
	}

	void SetSelectionFunction(void (*selectionFunctionPtr)(std::vector<T>&)) {
		selectFunction = selectionFunctionPtr;
	}

	void SetCrossoverFunction(void (*crossoverFunctionPtr)(T&, T&)) {
		crossoverFunction = crossoverFunctionPtr;
	}

	void SetMutationFunction(void (*mutationFunctionPtr)(T&)) {
		mutationFunction = mutationFunctionPtr;
	}

	T run() {
		initializePopulation();									//��ʼ����Ⱥ
		for (int i = 0; i < m_generations; ++i) {					//��������
			calculateFitness();									//������Ⱥ��Ӧ��
			if (selectFunction != nullptr) {
				selectFunction(population);
			}
			if (crossoverFunction != nullptr) {
				crossoverFunction()
			}
		}
	}

};


#endif
