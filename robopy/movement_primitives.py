import numpy as np
from scipy.linalg import block_diag
import time

from robopy.groups import Group
from robopy.trajectory import GoToTrajectory, NamedTrajectoryBase


def rbf(centers, width):
    b = lambda x: np.array([np.exp(-(x - c_i) ** 2 / (2 * h_i)) for c_i, h_i in zip(centers, width)]).T  # eq 7
    return lambda x: b(x) / np.sum(b(x), axis=1, keepdims=True)  # eq 8


def rbf_dot(centers, width):
    b_dot = lambda x: np.array([- np.exp(-(x - c_i) ** 2 / (2 * h_i)) * (x - c_i)/h_i
                                for c_i, h_i in zip(centers, width)]).T
    b = lambda x: np.array([np.exp(-(x - c_i) ** 2 / (2 * h_i)) for c_i, h_i in zip(centers, width)]).T  # eq 7

    first_term = lambda x: b_dot(x) / np.sum(b(x), axis=1, keepdims=True)
    second_term = lambda x: - b(x) / np.sum(b(x), axis=1, keepdims=True)**2 * np.sum(b_dot(x), axis=1, keepdims=True)
    return lambda x: first_term(x) + second_term(x)  # eq 8


class MovementSpace:

    def __init__(self, group, centers, bandwidths, regularization=1E-12):
        """
        :param group: On which group do you want to set your promp
        :type group: Group
        """
        self.n_features = len(centers)
        self.n_dim = len(group.refs)
        self.n_params = self.n_features * self.n_dim
        self.group = group
        self.centers = centers
        self.bandwidths = bandwidths
        self.phi = rbf(self.centers, self.bandwidths)
        self.phi_dot = rbf_dot(self.centers, self.bandwidths)
        self.l = regularization
        self.equal_phi = True

    def get_phi(self, z, dim=0):
        return self.phi(z)

    def get_phi_dot(self, z, dim=0):
        return self.phi_dot(z)

    def get_block_phi(self, z):
        ret = block_diag(*([self.get_phi(z, dim=0)]*self.n_dim))
        return ret

    def get_block_phi_dot(self, z):
        ret = block_diag(*([self.get_phi_dot(z, dim=0)]*self.n_dim))
        return ret

    def get_displacement(self, dim=0):
        return np.zeros(self.n_features)

    def get_block_displacement(self):
        return np.concatenate([self.get_displacement(i) for i in range(self.n_dim)])

    def get_trajectory_displacement(self, z, i):
        return np.matmul(MovementSpace.get_phi(self, z, i), self.get_displacement(i))

    def get_block_trajectory_displacement(self, z):
        return np.concatenate([self.get_trajectory_displacement(z, i)
                               for i in range(self.n_dim)], axis=0)


class ProjectedMovementSpace(MovementSpace):

    def __init__(self, group, centers, bandwidths, projection_matrix, displacement, regularization=1E-12):
        """
        :param group: On which group do you want to set your promp
        :type group: Group
        :param projection_matrix: n_parameters x n_components
        :type projection_matrix:np.nd_array
        :param displacement: n_parameters encoding the mean movement in this space
        :type displacement: np.nd_array
        """
        MovementSpace.__init__(self, group, centers, bandwidths, regularization)
        self.n_params = projection_matrix.shape[1]
        self.projection_matrix = projection_matrix
        self.displacement = displacement
        self.equal_phi = False

    def get_phi(self, z, dim=0):
        omega = self.projection_matrix[self.n_features * dim:self.n_features * (dim + 1), :]
        return np.matmul(self.phi(z), omega)

    def get_block_phi(self, z):
        ret = np.concatenate([self.get_phi(z, dim=i) for i in range(self.n_dim)], axis=0)
        return ret

    def get_displacement(self, dim=0):
        return self.displacement[self.n_features * dim: self.n_features * (dim + 1)]


class MovementPrimitive:

    def __init__(self, movement_space, parameters):
        """
        :param movement_space: the movement space of the movement primitive
        :type movement_space: MovementPrimitive
        :param parameters: a dictionary of the parameters for each dimension
        :type parameters: dict[np.nd_array]
        """
        self.movement_space = movement_space
        self.params = parameters

    def get_init_trajectory(self, duration=10.):
        z = np.array([0.])
        y = {ref: np.asscalar(np.matmul(self.movement_space.phi(z), self.params[ref]))
             for ref in self.movement_space.group.refs}
        return GoToTrajectory(duration, **y)

    @staticmethod
    def get_timing(frequency, duration):
        ctr_time = 1. / frequency
        n_points = int(duration * frequency)
        z = np.linspace(0., 1., n_points)
        return ctr_time, z, n_points

    def get_full_trajectory(self, frequency=20, duration=10.):
        ctr_time, z, n_points = MovementPrimitive.get_timing(frequency, duration)
        phi = self.movement_space.phi(z)
        y = np.array([np.matmul(phi, self.params[ref]) for ref in self.movement_space.group.refs]).T
        return NamedTrajectoryBase(self.movement_space.group.refs, np.array([ctr_time] * n_points), y)

    def get_full_trajectory_derivative(self, frequency=20, duration=10.):
        ctr_time, z, n_points = MovementPrimitive.get_timing(frequency, duration)
        phi_dot = self.movement_space.phi_dot(z)
        y = np.array([np.matmul(phi_dot, self.params[ref]) for ref in self.movement_space.group.refs]).T
        return NamedTrajectoryBase(self.movement_space.group.refs, np.array([ctr_time] * n_points), y)

    def get_block_params(self):
        return np.concatenate([self.params[ref] for ref in self.movement_space.group.refs], axis=0)

    @staticmethod
    def get_params_from_block(movement_space, w):
        """

        :param movement_space:
        :type movement_space: MovementSpace
        :param w:
        :return:
        """
        return {ref: w[movement_space.n_features * i:movement_space.n_features * (i + 1)]
                for i, ref in enumerate(movement_space.refs)}


class ProbabilisticMovementPrimitives:

    def __init__(self, movement_space, movements):
        """

        :param movement_space: movement space of the probabilistic movement primitive framework
        :type movement_space: MovementSpace
        :param movements: a list of movements
        :type movements: list[MovementPrimitive]
        """
        self.movement_space = movement_space
        self.movements = movements
        self._w = np.array([mp.get_block_params() for mp in self.movements])
        self.cov_w = np.cov(self._w.T)
        self.mean_w = np.mean(self._w, axis=0)

    def sample_movement(self):
        w = np.random.multivariate_normal(self.mean_w, self.cov_w)
        w = MovementPrimitive.get_params_from_block(self.movement_space, w)
        return MovementPrimitive(self.movement_space, w)


class PrincipalMovementPrimitive(MovementPrimitive):

    def __init__(self, movement_space, parameters):
        """

        :param movement_space:
        :type movement_space: ProjectedMovementSpace
        :param parameters:
        """
        MovementPrimitive.__init__(self, movement_space, parameters)

    def get_init_trajectory(self, duration=10.):
        z = np.array([0.])
        y = {ref: np.asscalar(
            self.movement_space.get_trajectory_displacement(z, i) +
            np.matmul(self.movement_space.get_phi(z, i), self.params))
             for i, ref in enumerate(self.movement_space.group.refs)}
        return GoToTrajectory(duration, **y)

    def get_full_trajectory(self, frequency=20, duration=10.):
        ctrl_time, z, n_points = MovementPrimitive.get_timing(frequency, duration)
        y_mean = np.array([self.movement_space.get_trajectory_displacement(z, i)
                           for i in range(self.movement_space.n_dim)])
        y_disp = np.array([np.matmul(self.movement_space.get_phi(z, i), self.params)
                  for i in range(self.movement_space.n_dim)])
        y = (y_mean + y_disp)
        return NamedTrajectoryBase(self.movement_space.group.refs, np.array([ctr_time] * n_points), y)

    def get_movement_primitive(self):
        w = self.movement_space.displacement + np.matmul(self.movement_space.projection_matrix, self.params)
        w_dict = {ref: w[self.movement_space.n_features*i:self.movement_space.n_features*(i+1)]
                for i, ref in enumerate(self.movement_space.group.refs)}
        return MovementPrimitive(self.movement_space, w_dict)

    def get_block_params(self):
        return self.params


def ClassicSpace(group, n_features=10, regularization=1E-12):
    h = (1. + 1 / n_features) / n_features  # 0.5 * (1. + 1 / n_features) / n_features  # TODO: why?
    h = h ** 2 * 0.5
    bandwidths = np.repeat([h], n_features, axis=0)
    centers = np.linspace(0 - 0.5 / n_features, 1 + 0.5 / n_features, n_features)
    return MovementSpace(group, centers, bandwidths, regularization / n_features)

# def ClassicSpace(group, n_features=10, regularization=1E-12):
#     h = 1/(np.sqrt(2.*n_features)*n_features)                       # TODO: why?
#     bandwidths = np.repeat([h], n_features, axis=0)
#     centers = np.linspace(-2 * h, (1 + 2 * h), n_features)
#     return MovementSpace(group, centers, bandwidths, regularization/n_features)


def LearnTrajectory(movement_space, trajectory):
    """

    :param movement_space:
    :type movement_space: MovementSpace
    :param trajectory:
    :type trajectory: NamedTrajectoryBase
    :return:
    """
    n = len(trajectory.duration)
    t = np.cumsum(trajectory.duration)
    t = (t - t[0]) / (t[-1] - t[0])

    bandwidths = movement_space.bandwidths
    centers = movement_space.centers
    l = movement_space.l

    corrected_penalty = n * movement_space.n_dim / movement_space.n_params
    if movement_space.equal_phi:
        start = time.time()
        phi = rbf(centers, bandwidths)

        Phi = phi(t)
        A = np.matmul(Phi.T, Phi) + corrected_penalty * l * np.eye(movement_space.n_features)
        w = {ref: np.linalg.solve(A, np.matmul(Phi.T, y)) for ref, y in zip(trajectory.refs, trajectory.values.T)}
        mp = MovementPrimitive(movement_space, w)
        print("Learn movement %f" % (time.time() - start))
    else:
        start = time.time()
        Phi = movement_space.get_block_phi(t)
        A = np.matmul(Phi.T, Phi) + corrected_penalty * l * np.eye(movement_space.n_params)
        values = trajectory.get_dict_values()
        y = np.concatenate([values[ref] for ref in movement_space.group.refs], axis=0)
        params = np.linalg.solve(A, np.matmul(Phi.T, (y - movement_space.get_block_trajectory_displacement(t))))
        mp = PrincipalMovementPrimitive(movement_space, params)
        print("Learn principal components %f" % (time.time() - start))
    return mp


def eigen_value_decomposition(cov_matrix):
    start = time.time()
    eigen_vals, eigen_vectors = [np.real(x) for x in np.linalg.eig(cov_matrix)]
    vals = np.abs(eigen_vals.reshape((len(eigen_vals), 1)))
    vectors = np.transpose(eigen_vectors)
    eigen_matrix = np.hstack((vals, vectors))
    eigen_matrix = np.flip(eigen_matrix[eigen_matrix[:, 0].argsort()], axis=0)
    print("Eigenvalues Decomposition %f" % (time.time() - start))
    return eigen_matrix


def LearnPrincipalMovements(movement_space, trajectories, n_components=5):
    movements = [LearnTrajectory(movement_space, t) for t in trajectories]
    
    promps = ProbabilisticMovementPrimitives(movement_space, movements)
    projection = eigen_value_decomposition(promps.cov_w)[:n_components, 1:].T
    projection_space = ProjectedMovementSpace(movement_space.group, movement_space.centers, movement_space.bandwidths,
                                              projection, promps.mean_w, regularization=movement_space.l)
    return projection_space, [LearnTrajectory(projection_space, t) for t in trajectories]
