#ifndef RTLBS_GA
#define RTLBS_GA

#include "rtlbs.h"
#include "utility/define.h"

template<typename T>
class GeneticAlgorithm {
private:
	int m_populationSize;							/** @brief	种群数量	*/
	int m_chromosomeLength;							/** @brief	染色体长度	*/
	double m_crossoverRate;							/** @brief	交叉率	*/
	double m_mutationRate;							/** @brief	变异率	*/
	int m_generations;								/** @brief	进化代数	*/

	std::vector<T> population;

	double (*fitnessFunction)(T);					//适应度函数指针

	void (* selectFunction)(std::vector<T>&);		//选择操作函数指针

	void (*crossoverFunction)(T&, T&);				//交叉操作函数指针

	void (*mutationFunction)(T&);					//变异操作函数指针

	


private:
	void initializePopulation() {					//初始化种群
		population.clear();
		for (int i = 0; i < m_populationSize; ++i) {
			T individual;
			population.push_back(individual);
		}
	}

	void calculateFitness() {						//计算种群适应度
		for (int i = 0; i < m_populationSize; ++i) {
			population[i].fitness = fitnessFunction(population[i]);
		}
	}

	void crossover() {								//基因交叉操作
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

	void mutate() {									//基因变异操作
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
		initializePopulation();									//初始化种群
		for (int i = 0; i < m_generations; ++i) {					//迭代次数
			calculateFitness();									//计算种群适应度
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
