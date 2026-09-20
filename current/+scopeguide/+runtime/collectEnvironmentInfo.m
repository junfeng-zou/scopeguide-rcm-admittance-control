function environment = collectEnvironmentInfo(projectRoot)
%COLLECTENVIRONMENTINFO Collect read-only Stage 0 runtime capabilities.

arguments
    projectRoot (1, 1) string
end

environment = struct();
environment.GeneratedAtUtc = string(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX'));
environment.ProjectRoot = projectRoot;
environment.MatlabVersion = string(version);
environment.MatlabRelease = string(version('-release'));
environment.Computer = string(computer);
environment.Architecture = string(computer('arch'));
environment.HasMatlabUnitTest = ...
    exist('matlab.unittest.TestCase', 'class') == 8;

environment.Optimization = struct();
environment.Optimization.QuadprogPath = string(which('quadprog'));
environment.Optimization.HasQuadprog = exist('quadprog', 'file') == 2;
environment.Optimization.LicenseAvailable = testLicense('Optimization_Toolbox');
environment.Optimization.QpRoute = chooseQpRoute(environment.Optimization);

environment.Network = struct();
environment.Network.HasTcpClient = exist('tcpclient', 'file') == 2;
environment.Network.HasModbus = exist('modbus', 'file') == 2;

products = ver;
environment.InstalledProducts = strings(numel(products), 1);
environment.InstalledProductVersions = strings(numel(products), 1);
for index = 1:numel(products)
    environment.InstalledProducts(index) = string(products(index).Name);
    environment.InstalledProductVersions(index) = string(products(index).Version);
end
end

function available = testLicense(featureName)
try
    available = logical(license('test', featureName));
catch
    available = false;
end
end

function route = chooseQpRoute(optimization)
if optimization.HasQuadprog && optimization.LicenseAvailable
    route = "quadprog";
else
    route = "scaled_dls_dry_run_only";
end
end
