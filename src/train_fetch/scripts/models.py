import torch
import torch.nn.functional as F

class ActorModel( torch.nn.Module ):
    def __init__( self, input_dims, output_dims, hidden_dims=1024 ):
        super( ActorModel, self ).__init__()
        self.linear1 = torch.nn.Linear( input_dims, hidden_dims )
        self.linear2 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.linear3 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.linear4 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.linear5 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.linear6 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.mu = torch.nn.Linear( hidden_dims, output_dims )
        self.sigma = torch.nn.Linear( hidden_dims, output_dims )

    def forward( self, x ):
        mid = F.relu( self.linear1( x ) )
        mid = F.relu( self.linear2( mid ) )
        mid = F.relu( self.linear3( mid ) )
        mid = F.relu( self.linear4( mid ) )
        mid = F.relu( self.linear5( mid ) )
        mid = F.relu( self.linear6( mid ) )
        mean = torch.clamp( self.mu( mid ), min=-6.28, max=6.28 )
        sigma = torch.clamp( self.sigma( mid ), min=1e-10, max=1 )
        return mean, sigma

    def getDistribution( self, x ):
        mu, sigma = self.forward( x )
        dist = torch.distributions.Normal( mu, sigma )
        return dist

class CriticModel( torch.nn.Module ):
    def __init__( self, input_dims, hidden_dims=100 ):
        super( CriticModel, self ).__init__()
        self.linear1 = torch.nn.Linear( input_dims, hidden_dims )
        self.linear2 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.linear3 = torch.nn.Linear( hidden_dims, hidden_dims )
        self.outputLinear = torch.nn.Linear( hidden_dims, 1 )

    def forward( self, x ):
        return  self.outputLinear( F.relu( self.linear3( F.relu( self.linear2( F.relu( self.linear1( x ) ) ) ) )  ) )

