import torch
import torch.nn.functional as F


def getModel(name, input_dims, output_dims):
    if name == "NAF":
        return NAFModel(input_dims, output_dims)
    if name == "decay":
        actor = ActorModelNoSigma(input_dims, output_dims)
        critic = CriticModel(input_dims)
        return ActorCriticModel(actor, critic)
    actor = ActorModel(input_dims, output_dims)
    critic = CriticModel(input_dims)
    return ActorCriticModel(actor, critic)


class ActorCriticModel(torch.nn.Module):
    def __init__(self, actor, critic):
        self.actor = actor
        self.critic = critic

    def forward(self, x):
        return self.actor(x)

    def getDistribution(self, x):
        return self.actor.getDistribution(x)

    def getValue(self, x, action):
        return self.critic(x)

    def decayStdDev(self, min_std_dev):
        self.actor.decayStdDev(min_std_dev)

    def setStdDev(self, std):
        self.actor.setStdDev(std)


class ActorModel(torch.nn.Module):
    def __init__(self, input_dims, output_dims, hidden_dims=1024):
        super(ActorModel, self).__init__()
        self.linear1 = torch.nn.Linear(input_dims, hidden_dims)
        self.linear2 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear3 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear4 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear5 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear6 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.mu = torch.nn.Linear(hidden_dims, output_dims)
        self.sigma = torch.nn.Linear(hidden_dims, output_dims)

    def forward(self, x):
        mid = F.relu(self.linear1(x))
        mid = F.relu(self.linear2(mid))
        mid = F.relu(self.linear3(mid))
        mid = F.relu(self.linear4(mid))
        mid = F.relu(self.linear5(mid))
        mid = F.relu(self.linear6(mid))
        mean = torch.clamp(self.mu(mid), min=-6.28, max=6.28)
        sigma = torch.clamp(self.sigma(mid), min=1e-10, max=1)
        return mean, sigma

    def getDistribution(self, x):
        mu, sigma = self.forward(x)
        dist = torch.distributions.Normal(mu, sigma)
        return dist


class CriticModel(torch.nn.Module):
    def __init__(self, input_dims, hidden_dims=100):
        super(CriticModel, self).__init__()
        self.linear1 = torch.nn.Linear(input_dims, hidden_dims)
        self.linear2 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear3 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.outputLinear = torch.nn.Linear(hidden_dims, 1)

    def forward(self, x):
        return self.outputLinear(
            F.relu(self.linear3(F.relu(self.linear2(F.relu(self.linear1(x))))))
        )


class ActorModelNoSigma(torch.nn.Module):
    _decay_rate = 0.95
    _sigma = 1.57

    def __init__(self, input_dims, output_dims, hidden_dims=1024):
        super(ActorModelNoSigma, self).__init__()
        self.linear1 = torch.nn.Linear(input_dims, hidden_dims)
        self.linear2 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear3 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear4 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear5 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear6 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.mu = torch.nn.Linear(hidden_dims, output_dims)

    def forward(self, x):
        mid = F.relu(self.linear1(x))
        mid = F.relu(self.linear2(mid))
        mid = F.relu(self.linear3(mid))
        mid = F.relu(self.linear4(mid))
        mid = F.relu(self.linear5(mid))
        mid = F.relu(self.linear6(mid))
        mean = torch.clamp(self.mu(mid), min=-6.28, max=6.28)
        return mean

    def decayStdDev(self, min_std_dev):
        if self._sigma == min_std_dev:
            self._sigma = min_std_dev
        else:
            self._sigma = max(self._sigma * self._decay_rate, min_std_dev)

    def setStdDev(self, std):
        self._sigma = std

    def getDistribution(self, x):
        mu = self.forward(x)
        dist = torch.distributions.Normal(mu, torch.full(mu.shape, self._sigma))
        return dist


class NAFModel(torch.nn.Module):
    def __init__(self, input_dims, output_dims, hidden_dims=1024):
        super(ActorModelNoSigma, self).__init__()
        self.linear1 = torch.nn.Linear(input_dims, hidden_dims)
        self.linear2 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear3 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear4 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear5 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear6 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.mu = torch.nn.Linear(hidden_dims, output_dims)
        self.value = torch.nn.Linear(hidden_dims, 1)
        self.entries = torch.nn.Linear(
            hidden_dims, int(output_dims * (output_dims + 1) / 2)
        )

        self.output_dims = output_dims

    def forward_prop(self, x, action=None):
        mid = F.relu(self.linear1(x))
        mid = F.relu(self.linear2(mid))
        mid = F.relu(self.linear3(mid))
        mid = F.relu(self.linear4(mid))
        mid = F.relu(self.linear5(mid))
        d = F.relu(self.linear6(mid))
        mean = torch.clamp(self.mu(mid), min=-6.28, max=6.28)
        Value = self.value(mid)
        entries = torch.tanh(self.entries(mid))

        action_value = mu.unsqueeze(-1)
        L = torch.zeros((x.shape[0], self.output_dims, self.output_dims))
        tril_indices = torch.tril_indices(
            row=self.output_dims, col=self.output_dims, offset=0
        )
        L[:, tril_indices[0], tril_indices[1]] = entries
        L.diagonal(dim1=1, dim2=2).exp_()
        P = L * L.transpose(2, 1)
        Q = None

        if action is not None:
            dist = MultivariateNormal(action_value.squeeze(-1), torch.inverse(P))
            action = torch.clamp(dist.sample(), min=-6.28, max=6.28)
            A = (
                -0.5
                * torch.matmul(
                    torch.matmul(
                        (action.unsqueeze(-1) - action_value).transpose(2, 1), P
                    ),
                    (action.unsqueeze(-1) - action_value),
                )
            ).squeeze(-1)
            Q = A + V

        dist = MultivariateNormal(action_value.squeeze(-1), torch.inverse(P))

        return action_value, dist, Q, V

        def forward(self, x):
            action, _, _, _ = self.forward_prop(self, x)
            return action

        def getDistribution(self, x, action):
            _, dist, _, _ = self.forward_prop(x, action)
            return dist

        def getQValue(self, x, action):
            _, _, Q, _ = self.forward_prop(x, action)
            return Q

        def getValue(self, x, action):
            _, _, _, V = self.forward_prop(x, action)
            return V
