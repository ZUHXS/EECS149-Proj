# Variables used to override the disposition of source tree
$ENV{CIXCVR_COMMON_ROOT} = "common" if $ENV{CIXCVR_COMMON_ROOT} eq "";
$ENV{CIXCVR_UMTS_RACFL_PRODUCT_ROOT} = "product/umts/rac" if $ENV{CIXCVR_UMTS_RACFL_PRODUCT_ROOT} eq "";
$ENV{CIXCVR_UMTS_RACFL_SYSTEMTEST_ROOT} = "systemtest/product/umts/rac" if $ENV{CIXCVR_UMTS_RACFL_SYSTEMTEST_ROOT} eq 
"";
%componentDefinition = 
(
  "type"               => "INTERFACE",
  "desc"               => "CI Common Includes",
  "targetName"         => "Common",
  "supportedPlatforms" => ["himalaya", "himalaya_be", "laplace", "laplace_be", "c64plus","c64pluse", "c6488", "c6488e", "c6498", "c6498e", "gcc"],
  "supportedProfiles"  => ["release", "debug"],
  "pkgRoot"            => "$ENV{CIXCVR_COMMON_ROOT}/api",
); 


%includePath =
( 
  "c64plus"    => [ ".", "c6x" ],
  "c64pluse"    => [ ".", "c6x" ],
  "c6488"    => [ ".", "c6x" ],
  "c6488e"    => [ ".", "c6x" ],
  "c6498"    => [ ".", "c6x" ],
  "c6498e"    => [ ".", "c6x" ],
  "himalaya" => [".", "c6x"],
  "himalaya_be" => [".", "c6x"],
  "laplace" => [".", "c6x"],
  "laplace_be" => [".", "c6x"],
  "gcc" => [".", "c6x"],
);


