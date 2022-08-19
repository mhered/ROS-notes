import math


def phi(L1, L2, alpha):

    L3 = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(alpha))
    cos_phi = L2*math.sin(alpha)/L3

    # round to avoid numerical errors
    phi = math.degrees(math.acos(round(cos_phi, 8)))

    # by convention when wall on the left phi > 0 inwards, i.e. ccwise
    if L2 > L1 / math.cos(alpha):
        return phi
    else:
        return -phi


if __name__ == "__main__":

    test_cases = [{'r': 1.0,
                   'r_alpha': 0.5*math.sqrt(2),
                  'alpha': math.radians(45),
                   'phi_result': -45.0,
                   },
                  {'r': 0.6,
                  'r_alpha': 0.6/math.cos(math.radians(30.0)),
                   'alpha': math.radians(30.0),
                   'phi_result': 0.0,
                   },
                  {'r': 2.0,
                  'r_alpha': 2.0*math.cos(math.radians(30.0)),
                   'alpha': math.radians(30.0),
                   'phi_result': -30.0,
                   },
                  ]

    for item in test_cases:
        phi_i = phi(item['r'], item['r_alpha'], item['alpha'])
        print(f"phi={repr(phi_i)}")
        should_be_msg = f"should be {item['phi_result']}"
        assert round(phi_i, 5) == item['phi_result'], should_be_msg


# at end of wall? -> rotate 78deg
print(phi(0.6, 5, math.radians(10)))

# keep straight
print(phi(0.6, .60926, math.radians(10)))

# 5deg inwards
print(phi(0.6, .6188, math.radians(10)))

# 5 deg outwards
print(phi(0.6, .6, math.radians(10)))

# 10deg inwards
print(phi(0.6, .629, math.radians(10)))

# 10 deg outwards
print(phi(0.6, .5906, math.radians(10)))
