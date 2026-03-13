import numpy as np
import pandas as pd
from typing import Tuple, Dict, Any, Optional
import yaml


class PacejkaTireModel:
    """
    Simplified Pacejka Magic Formula tire model.
        
    Parameters:
    -----------
    Longitudinal (x):
        pCx1 : Shape factor C
        pDx1 : Peak factor D (at Fz0)
        pDx2 : Peak factor D variation with load
        pEx1 : Curvature factor E
        pKx1 : Slip stiffness K
        pKx3 : Slip stiffness variation with load (exponential)
        
    Lateral (y):
        pCy1 : Shape factor C
        pDy1 : Peak factor D (at Fz0)
        pDy2 : Peak factor D variation with load
        pEy1 : Curvature factor E
        pKy1 : Cornering stiffness K
        pKy2 : Cornering stiffness load dependency
        
    Common:
        Fz0  : Nominal load [N]
        lambda_mu_x : Longitudinal friction scaling factor
        lambda_mu_y : Lateral friction scaling factor
    """
    
    # Default parameters for 43100 R20 tire
    DEFAULT_PARAMS = {
        'pCx1': 1.7,
        'pDx1': 1.26,
        'pDx2': -0.5,
        'pEx1': -2,
        'pKx1': 16.12,
        'pKx3': -0.05,
        'pCy1': 1.13,
        'pDy1': 2.5,
        'pDy2': -0.37,
        'pEy1': 0.419,
        'pKy1': 49,
        'pKy2': 2.22,
        'Fz0': 1051,
        'lambda_mu_x': 1.5,
        'lambda_mu_y': 0.9745,
    }
    
    def __init__(self, params: Optional[Dict[str, float]] = None):
        """
        Initialize the tire model with parameters.
        
        Parameters:
        -----------
        params : Dictionary of model parameters. Missing parameters
                 will be filled with defaults.
        """
        self.params = self.DEFAULT_PARAMS.copy()
        if params is not None:
            self.params.update(params)
    
    def __repr__(self):
        return (f"PacejkaTireModel(Fz0={self.params['Fz0']:.0f}N, "
                f"μx≈{self.params['pDx1']:.2f}, μy≈{self.params['pDy1']:.2f})")

    def compute_practical_slips(self, kappa: np.ndarray, alpha: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute practical slip quantities.
        
        Parameters:
        -----------
        kappa : Longitudinal slip ratio (dimensionless, e.g., 0.1 = 10%)
        alpha : Slip angle [radians]
        
        Returns:
        --------
        sigma_x, sigma_y, sigma_tot
        """
        denom = np.sqrt(1 + kappa**2)
        sigma_x = kappa / denom
        sigma_y = np.tan(alpha) / denom
        sigma_tot = np.sqrt(sigma_x**2 + sigma_y**2)

        # Avoid division by zero singularity
        sigma_tot = np.maximum(sigma_tot, 1e-10)
        return sigma_x, sigma_y, sigma_tot
    
    def compute_x_coeff(self, Fz: np.ndarray) -> Tuple[np.ndarray, ...]:
        """
        Compute longitudinal Pacejka coefficients B, C, D, E.
        
        Parameters:
        -----------
        Fz : Normal load [N]
        
        Returns:
        --------
        Bx, Cx, Dx, Ex
        """
        p = self.params
        
        Fz0 = p['Fz0']
        dfz = (Fz - Fz0) / Fz0  # normal load scaling
        
        # Slip stiffness Kx [N/unit slip] - note: multiply by Fz0 for proper scaling
        Kx = p['pKx1'] * Fz0 * np.exp(p['pKx3'] * dfz)
        Ex = p['pEx1'] * np.ones_like(Fz)
        Dx = (p['pDx1'] + p['pDx2'] * dfz) * p['lambda_mu_x']
        Dx = np.maximum(Dx, 0.01)  # Ensure Dx is positive
        Cx = p['pCx1'] * np.ones_like(Fz)
        Bx = Kx / (Cx * Dx * Fz + 1e-10)
        
        return Bx, Cx, Dx, Ex
    
    def compute_y_coeff(self, Fz: np.ndarray) -> Tuple[np.ndarray, ...]:
        """
        Compute lateral Pacejka coefficients B, C, D, E.
        
        Parameters:
        -----------
        Fz : Normal load [N]
        
        Returns:
        --------
        By, Cy, Dy, Ey
        """
        p = self.params
        Fz0 = p['Fz0']
        dfz = (Fz - Fz0) / Fz0
        
        Ky = p['pKy1'] * Fz0 * np.sin(2 * np.arctan(Fz / (p['pKy2'] * Fz0)))  # Cornering stiffness
        Ey = p['pEy1'] * np.ones_like(Fz)
        Dy = (p['pDy1'] + p['pDy2'] * dfz) * p['lambda_mu_y']
        Dy = np.maximum(Dy, 0.01)  # ensure positive
        Cy = p['pCy1'] * np.ones_like(Fz)
        By = Ky / (Cy * Dy * Fz + 1e-10)
        
        return By, Cy, Dy, Ey
    
    def magic_formula(self, sigma: np.ndarray, B: np.ndarray, C: np.ndarray, 
                      D: np.ndarray, E: np.ndarray) -> np.ndarray:
        """
        Pacejka Magic Formula shape function.
        
        F/Fz = D * sin(C * arctan(B*sigma - E*(B*sigma - arctan(B*sigma))))
        """
        Bs = B * sigma
        return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))
    
    def compute_forces(self, Fz: np.ndarray, kappa: np.ndarray, 
                       alpha: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute combined tire forces.
        
        Parameters:
        -----------
        Fz : Normal load [N]
        kappa : Longitudinal slip ratio (dimensionless)
        alpha : Slip angle [radians]
        
        Returns:
        --------
        Fx, Fy : Longitudinal and lateral forces [N]
        """
        Fz = np.atleast_1d(Fz).astype(float)
        kappa = np.atleast_1d(kappa).astype(float)
        alpha = np.atleast_1d(alpha).astype(float)
        
        # Compute practical slips
        sigma_x, sigma_y, sigma_tot = self.compute_practical_slips(kappa, alpha)
        
        # Coefficients
        Bx, Cx, Dx, Ex = self.compute_x_coeff(Fz)
        By, Cy, Dy, Ey = self.compute_y_coeff(Fz)
        
        # Magic formula
        mu_x = self.magic_formula(sigma_tot, Bx, Cx, Dx, Ex)
        mu_y = self.magic_formula(sigma_tot, By, Cy, Dy, Ey)
        
        # Forces with slip direction weighting
        Fx = Fz * (sigma_x / sigma_tot) * mu_x
        Fy = Fz * (sigma_y / sigma_tot) * mu_y
        
        return Fx, Fy
    
    def compute_pure_lateral(self, Fz: np.ndarray, alpha: np.ndarray) -> np.ndarray:
        """
        Compute pure lateral force (kappa=0).
        
        Parameters:
        -----------
        Fz : Normal load [N]
        alpha : Slip angle [radians]
        
        Returns:
        --------
        Fy : Lateral force [N]
        """
        kappa = np.zeros_like(np.atleast_1d(alpha))
        _, Fy = self.compute_forces(Fz, kappa, alpha)
        return Fy
    
    def compute_pure_longitudinal(self, Fz: np.ndarray, kappa: np.ndarray) -> np.ndarray:
        """
        Compute pure longitudinal force (alpha=0).
        
        Parameters:
        -----------
        Fz : Normal load [N]
        kappa : Slip ratio [-]
        
        Returns:
        --------
        Fx : Longitudinal force [N]
        """
        alpha = np.zeros_like(np.atleast_1d(kappa))
        Fx, _ = self.compute_forces(Fz, kappa, alpha)
        return Fx
    
    def to_dict(self) -> Dict[str, float]:
        """Return parameters as dictionary."""
        return self.params.copy()
    
    
    def scale_friction(self, mu_x: float = None, mu_y: float = None):
        """Scale friction coefficients for this instance."""
        if mu_x is not None:
            self.params['lambda_mu_x'] = mu_x
        if mu_y is not None:
            self.params['lambda_mu_y'] = mu_y
        return self
    
    def plot_model(self, Fz_levels=None, n_points=100, *, show=True, save_path=None):
        """Plot pure lateral, pure longitudinal"""
        from wrapper import Figure, Line

        Fz0 = self.params['Fz0']
        if Fz_levels is None:
            Fz_levels = [0.5*Fz0, 0.75*Fz0, Fz0, 1.25*Fz0]

        alpha_sweep = np.linspace(-np.radians(15), np.radians(15), n_points)
        kappa_sweep = np.linspace(-0.3, 0.3, n_points)
        colors = ['#440154', '#31688e', '#35b779', '#fde725'][:len(Fz_levels)]
        fig = Figure(nrows=1, ncols=2)

        for fz, c in zip(Fz_levels, colors):
            label = f'{fz:.0f} N'
            Fy = self.compute_pure_lateral(fz, alpha_sweep)
            Fx = self.compute_pure_longitudinal(fz, kappa_sweep)

            fig.add(Line(np.degrees(alpha_sweep), Fy, color=c, linewidth=2, label=label), subplot=0)
            fig.add(Line(kappa_sweep, Fx, color=c, linewidth=2, label=label), subplot=1)

        fig.set_labels(0, xlabel='Slip Angle [deg]', ylabel='Fy [N]', title='Pure Lateral')
        fig.set_labels(1, xlabel='Slip Ratio [-]', ylabel='Fx [N]', title='Pure Longitudinal')

        for i in range(2):
            fig.legend(i, fontsize=7)
            fig.grid(i)

        fig.render()
        if save_path:
            fig.save(save_path, dpi=180)
        if show:
            fig.show()
        else:
            fig.close()
        return fig
    
    @classmethod
    def from_yaml(cls, filename: str) -> 'PacejkaTireModel':
        """Load parameters from YAML file."""
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
        
        pm = data['pacejka_tire_model_coeff']
        params = {}
        params.update(pm['longitudinal'])
        params.update(pm['lateral'])
        params.update(pm['common'])
        
        return cls(params)