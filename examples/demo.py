from eco_cps import Scenario, simulate

if __name__ == "__main__":
    result = simulate("joint", Scenario(), seed=0)
    print(result.summary())
