#############################################################################
#
# (C) 2021 Cadence Design Systems, Inc. All rights reserved worldwide.
#
# This sample script is not supported by Cadence Design Systems, Inc.
# It is provided freely for demonstration purposes only.
# SEE THE WARRANTY DISCLAIMER AT THE BOTTOM OF THIS FILE.
#
#############################################################################

#
# ============================================================================
# GENERATE AN UNSTRUCTURED VOLUME MESH FOR A GENERIC AIRCRAFT GEOMETRY
# ============================================================================
# Written by: Zach Davis, Pointwise, Inc.
#
# This script demonstrates the viscous meshing process using a generic 
# transport aircraft geometry created with OpenVSP. The user has control 
# over the farfield size and boundary layer resolution. The resulting grid
# is saved to the current working directory. 
#

# --------------------------------------------------------
# -- INITIALIZATION
# --
# -- Load Glyph package, initialize Pointwise, and
# -- define the working directory.
# --
# --------------------------------------------------------

# Load Glyph and Tcl Data Structure Libraries
package require PWI_Glyph

# Initialize Pointwise
pw::Application reset
pw::Application clearModified

# Define Working Directory
set scriptDir [file dirname [info script]]

# --------------------------------------------------------
# -- USER-DEFINED PARAMETERS
# --
# -- Define a set of user-controllable settings.
# --
# --------------------------------------------------------

set fileName     "GenericTransport.pw";  # Aircraft geometry filename
set cLen                         238.0;  # Characteristic length (inches)
set ffSize                       100.0;  # Farfield size (cLen multiplier)
set avgDs1                         8.0;  # Initial surface triangle average edge length
set avgDs2                         4.0;  # Refined surface triangle average edge length
set avgDs3        [expr {5.0 * $cLen}];  # Farfield surface triangle average edge length
set teDim                            4;  # Number of points across trailing edges
set rtLayers                        21;  # Root/Tip connector distribution layers 
set rtGrowthRate                   1.1;  # Root/Tip connector distribution growth rate
set leteLayers                      30;  # Leading/Trailing edge connector distribution layers
set leteGrowthRate                 1.1;  # Leading/Trailing edge connector distribution layers
set domLayers                       15;  # Layers for 2D T-Rex surface meshing
set domGrowthRate                  1.2;  # Growth rate for 2D T-Rex surface meshing
set aspectRatio                   10.0;  # Aspect ratio for 2D T-Rex surface meshing
set initDs                      0.0015;  # Initial wall spacing for boundary layer extrusion
set growthRate                     1.2;  # Growth rate for boundary layer extrusion
set boundaryDecay                 0.85;  # Volumetric boundary decay
set numLayers                      100;  # Max number of layers to extrude
set fullLayers                       1;  # Full layers (0 for multi-normals, 1 for single normal)
set collisionBuffer                  2;  # Collision buffer for colliding fronts
set maxAngle                     165.0;  # Max included angle for boundary elements
set centroidSkew                   1.0;  # Max centroid skew for boundary layer elements

# --------------------------------------------------------
# -- PROCEDURES
# --
# -- Define procedures for frequent tasks. 
# --
# --------------------------------------------------------

# Get Domains from Underlying Quilt
proc DomFromQuilt { quilt } {
    set gridsOnQuilt [$quilt getGridEntities]

    foreach grid $gridsOnQuilt {
        if { [$grid isOfType pw::DomainUnstructured] } {
            lappend doms $grid
        }
    }

    if { ![info exists doms] } {
        puts "INFO: There are no domains on [$quilt getName]"
        return
    }

    return $doms
}

# Get All Connectors in a Domain
proc ConsFromDom { dom } {
    set numEdges [$dom getEdgeCount]

    for { set i 1 } { $i <= $numEdges } { incr i } {
        set edge [$dom getEdge $i]
        set numCons [$edge getConnectorCount]

        for { set j 1 } { $j <= $numCons } {incr j} {
            lappend cons [$edge getConnector $j]
        }
    }

    return $cons
}

# Apply a Growth Distribution to Selected Connectors
proc RedistCons { rate layers conList } {
    global avgDs2 aspectRatio
    set conMode [pw::Application begin Modify $conList]
        foreach con $conList {
            $con replaceDistribution 1 [pw::DistributionGrowth create]
            set dist [$con getDistribution 1]

            $dist setBeginSpacing [expr {$avgDs2/$aspectRatio}]
            $dist setBeginMode    LayersandRate
            $dist setBeginRate    $rate
            $dist setBeginLayers  $layers

            $dist setEndSpacing [expr {$avgDs2/$aspectRatio}]
            $dist setEndMode    LayersandRate
            $dist setEndRate    $rate
            $dist setEndLayers  $layers

            $con setDimensionFromDistribution
        }
    $conMode end
    unset conMode
}

# Make a Hemisphere of Unstructured Domains
proc UnsHemisphere { diameter center spacing } {
    set dZ  [expr {[lindex $center 2] - $diameter}]
    set cp1 [list [lindex $center 0] [lindex $center 1] $dZ]
    set dZ  [expr {[lindex $center 2] + $diameter}]
    set cp2 [list [lindex $center 0] [lindex $center 1] $dZ]

    set dbCurve [pw::Curve create]

        set circSeg [pw::SegmentCircle create]
            $circSeg addPoint $cp1
            $circSeg addPoint $cp2
            $circSeg setCenterPoint $center "0.0 -1.0 0.0"
    $dbCurve addSegment $circSeg
    $dbCurve setName "outer-circle"

    set hemiSphere [pw::Surface create]
        $hemiSphere revolve -angle 180.0 $dbCurve $cp1 "0.0 0.0 -1.0"
        $hemiSphere setName "outer-sphere"

    pw::Connector setCalculateDimensionSpacing $spacing
    set hemiDoms [pw::DomainUnstructured createOnDatabase [list $hemiSphere]]

    for { set i 0 } { $i < [llength $hemiDoms] } { incr i } {
        set dom [lindex $hemiDoms $i]
        $dom setRenderAttribute LineMode "Boundary"
        set edge [$dom getEdge 1]

        for { set j 1 } { $j <= [$edge getConnectorCount] } { incr j } {
            lappend hemiCons($i) [$edge getConnector $j]
        }
    }

    set symCons [symdiff $hemiCons(0) $hemiCons(1)]

    return [list $hemiDoms $symCons]
}

# Computes the Set Containing the Intersection of Set1 & Set2
proc intersect { set1 set2 } {
    set set3 [list]

    foreach item $set1 {
        if { [lsearch -exact $set2 $item] >= 0 } {
            lappend set3 $item
        }
    }

    return $set3
}

# Computes the Set Containing the Difference of Set1 and Set2
proc difference { set1 set2 } {
    set set3 [list]

    foreach item $set1 {
        if { [lsearch -exact $set2 $item] < 0 } {
            lappend set3 $item
        }
    }

    return $set3
}

# Computes the Set Containing the Symmetric Difference of Set1 & Set2
proc symdiff { set1 set2 } {
    set set3 [list]

    foreach item $set1 {
        if { [lsearch -exact $set2 $item] < 0 } {
            lappend set3 $item
        }
    }

    foreach item $set2 {
        if { [lsearch -exact $set1 $item] < 0 } {
            lappend set3 $item
        }
    }

    return $set3
}

# Query the System Clock
proc timestamp {} {
    puts [clock format [clock seconds] -format "%a %b %d %Y %l:%M:%S%p %Z"]
}

# Query the System Clock (ISO 8601 formatting)
proc timestamp_iso {} {
    puts [clock format [clock seconds] -format "%G-%m-%dT%T%Z"]
}

# Convert Time in Seconds to h:m:s Format
proc convSeconds { time } {
    set h [expr { int(floor($time/3600)) }]
    set m [expr { int(floor($time/60)) % 60 }]
    set s [expr { int(floor($time)) % 60 }]
    return [format  "%02d Hours %02d Minutes %02d Seconds" $h $m $s]
}

# --------------------------------------------------------
# -- MAIN ROUTINE
# --
# -- Main meshing procedure:
# --   Load Pointwise File
# --   Apply User Settings
# --   Gather Analysis Model Information
# --   Mesh Surfaces
# --   Refine Wing and Tail Surface Meshes with 2D T-Rex
# --   Refine Fuselage Surface Mesh
# --   Create Farfield
# --   Create Symmetry Plane
# --   Create Farfield Block
# --   Examine Blocks
# --   CAE Setup
# --   Save the Pointwise Project
# --------------------------------------------------------


# Start Time
set tBegin [clock seconds]
timestamp

# Load Pointwise Project File Containing Prepared OpenVSP Geometry
pw::Application load [file join $scriptDir $fileName]
pw::Display update

# Apply User Settings
pw::Connector setCalculateDimensionMethod Spacing
pw::Connector setCalculateDimensionSpacing $avgDs1

pw::DomainUnstructured setDefault BoundaryDecay $boundaryDecay
pw::DomainUnstructured setDefault Algorithm AdvancingFront

# Gather Analysis Model Information
set allDbs [pw::Database getAll]
foreach db $allDbs {
    if { [$db getDescription] == "Model"} {
        set dbModel $db
    }
}

set numQuilts [$dbModel getQuiltCount]
for { set i 1 } { $i <= $numQuilts } { incr i } {
    lappend dbQuilts [$dbModel getQuilt $i]
}

pw::Display update

# Mesh Surfaces
if { [$dbModel isBaseForDomainUnstructured] } {
    puts "Meshing [$dbModel getName] model..."
    set surfDoms [pw::DomainUnstructured createOnDatabase -joinConnectors 30 \
        -reject unusedSurfs $dbModel]

    if { [llength $unusedSurfs] > 0 } {
        puts "Unused surfaces exist, please check geometry."
        puts $unusedSurfs
        exit -1
    }
} else {
    puts "Unable to mesh model."
    exit -1
}

# Survey Surface Mesh and Isolate Domains and Connectors
foreach qlt $dbQuilts {
    set modelDoms([$qlt getName]) [DomFromQuilt $qlt]
}

set wingLowerCons [ConsFromDom $modelDoms(wing-lower)]
set wingUpperCons [ConsFromDom $modelDoms(wing-upper)]
set wingTipCons   [ConsFromDom $modelDoms(wing-tip)]
set wingTECons    [ConsFromDom $modelDoms(wing-trailing-edge)]

set horizTailLowerCons [ConsFromDom $modelDoms(htail-lower)]
set horizTailUpperCons [ConsFromDom $modelDoms(htail-upper)]
set horizTailTipCons   [ConsFromDom $modelDoms(htail-tip)]
set horizTailTECons    [ConsFromDom $modelDoms(htail-trailing-edge)]

set vertTailCons       [ConsFromDom $modelDoms(vtail)]
set vertTailTipCons    [ConsFromDom $modelDoms(vtail-tip)]
set vertTailTECons     [ConsFromDom $modelDoms(vtail-trailing-edge)]

set fairingCons [ConsFromDom $modelDoms(fairing)]

set fuselageCons [ConsFromDom $modelDoms(fuselage)]

# Update Display Window to More Clearly Render the Surface Mesh
pw::Display setShowDatabase 0

set allDomsCollection [pw::Collection create]
    $allDomsCollection set [pw::Grid getAll -type pw::Domain]
    $allDomsCollection do setRenderAttribute FillMode HiddenLine
$allDomsCollection delete

pw::Display update

# Refine Wing and Tail Surface Meshes with 2D T-Rex

## Adjust Domain Solver Attributes/Update Edge Spacing on Wing & Tail Connectors
set wingTailDomsCollection [pw::Collection create]
    $wingTailDomsCollection set [list $modelDoms(wing-lower)  \
                                      $modelDoms(wing-upper)  \
                                      $modelDoms(htail-lower) \
                                      $modelDoms(htail-upper) \
                                      $modelDoms(vtail)]
    $wingTailDomsCollection do setUnstructuredSolverAttribute \
        EdgeMaximumLength $avgDs1
$wingTailDomsCollection delete

set wingTailConsCollection [pw::Collection create]
    $wingTailConsCollection set [lsort -unique [join [list $wingLowerCons      \
                                                           $wingUpperCons      \
                                                           $wingTipCons        \
                                                           $horizTailLowerCons \
                                                           $horizTailUpperCons \
                                                           $horizTailTipCons   \
                                                           $vertTailCons       \
                                                           $vertTailTipCons    \
                                                           $fairingCons]]]
   pw::Connector setCalculateDimensionSpacing $avgDs2
   $wingTailConsCollection do calculateDimension
$wingTailConsCollection delete

## Isolate Root, Tip, & LE/TE Connectors on Wing/Tail for 2D T-Rex
set wingLECon        [intersect $wingLowerCons $wingUpperCons]
set wingTELowerCon   [intersect $wingLowerCons $wingTECons]
set wingTEUpperCon   [intersect $wingUpperCons $wingTECons]
set wingLowerRootCon [intersect $wingLowerCons $fairingCons]
set wingUpperRootCon [intersect $wingUpperCons $fairingCons]
set wingLowerTipCon  [intersect $wingLowerCons $wingTipCons]
set wingUpperTipCon  [intersect $wingUpperCons $wingTipCons]
set wingRootTECon    [intersect $wingTECons $fairingCons]
set wingTipTECon     [intersect $wingTECons $wingTipCons]

set horizTailLECon        [intersect $horizTailLowerCons \
                                     $horizTailUpperCons]
set horizTailTELowerCon   [intersect $horizTailLowerCons \
                                     $horizTailTECons]
set horizTailTEUpperCon   [intersect $horizTailUpperCons \
                                     $horizTailTECons]
set horizTailLowerRootCon [intersect $horizTailLowerCons \
                                     $fuselageCons]
set horizTailUpperRootCon [intersect $horizTailUpperCons \
                                     $fuselageCons]
set horizTailLowerTipCon  [intersect $horizTailLowerCons \
                                     $horizTailTipCons]
set horizTailUpperTipCon  [intersect $horizTailUpperCons \
                                     $horizTailTipCons]
set horizTailRootTECon    [intersect $horizTailTECons    \
                                     $fuselageCons]
set horizTailTipTECon     [intersect $horizTailTECons    \
                                     $horizTailTipCons]

set vertTailRootCon       [intersect $fuselageCons $vertTailCons]
set vertTailLowerTipCon   [intersect $vertTailCons       \
                                     $vertTailTipCons]
set vertTailRootTECon     [intersect $vertTailTECons $fuselageCons]
set vertTailTipTECon      [intersect $vertTailTipCons    \
                                     $vertTailTECons]
set vertTailTEUpperCon    [intersect $vertTailCons $vertTailTECons]

foreach con $vertTailCons {
    if { ![$con equals $vertTailRootCon] && \
         ![$con equals $vertTailLowerTipCon] && \
         ![$con equals $vertTailTEUpperCon] } {
             set vertTailLECon $con
    }
}

foreach con $vertTailTECons {
    if { ![$con equals $vertTailTEUpperCon] && \
         ![$con equals $vertTailRootTECon]  && \
         ![$con equals $vertTailTipTECon] } {
             set vertTailTELowerCon $con
    }
}

foreach con $vertTailTipCons {
    if { ![$con equals $vertTailLowerTipCon] && \
         ![$con equals $vertTailTipTECon] } {
             set vertTailUpperTipCon $con
    }
}

pw::Display update

## Modify Connector Distributions at the Wing & Tail Root/Tip
set chordCons [list $wingLowerRootCon      \
                    $wingUpperRootCon      \
                    $horizTailLowerRootCon \
                    $horizTailUpperRootCon \
                    $vertTailRootCon       \
                    $wingLowerTipCon       \
                    $wingUpperTipCon       \
                    $horizTailLowerTipCon  \
                    $horizTailUpperTipCon  \
                    $vertTailLowerTipCon   \
                    $vertTailUpperTipCon]

RedistCons $rtGrowthRate $rtLayers $chordCons

## Initialize Wing & Tail Tip Domains
set tipDomsCollection [pw::Collection create]
    $tipDomsCollection set [list $modelDoms(wing-tip)  \
                                 $modelDoms(htail-tip) \
                                 $modelDoms(vtail-tip)]
    $tipDomsCollection do initialize
$tipDomsCollection delete

pw::Display update

## Modify Connector Distributions at Wing/Tail Leading & Trailing Edges
set spanCons [list $wingLECon           \
                   $wingTELowerCon      \
                   $wingTEUpperCon      \
                   $horizTailLECon      \
                   $horizTailTELowerCon \
                   $horizTailTEUpperCon \
                   $vertTailLECon       \
                   $vertTailTELowerCon  \
                   $vertTailTEUpperCon]

RedistCons $leteGrowthRate $leteLayers $spanCons

## Modify Connector Dimensions at TE for Wing/Tail
set trailingEdgeRootConsCollection [pw::Collection create]
    $trailingEdgeRootConsCollection set [list $wingRootTECon      \
                                              $wingTipTECon       \
                                              $horizTailRootTECon \
                                              $horizTailTipTECon  \
                                              $vertTailRootTECon  \
                                              $vertTailTipTECon]
    $trailingEdgeRootConsCollection do setDimension $teDim
$trailingEdgeRootConsCollection delete

## Re-create Unstructured Domains on Wing/Tail Trailing Edges as Structured
set idx [lsearch $surfDoms $modelDoms(wing-trailing-edge)]
set surfDoms [lreplace $surfDoms $idx $idx]
set idx [lsearch $surfDoms $modelDoms(htail-trailing-edge)]
set surfDoms [lreplace $surfDoms $idx $idx]
set idx [lsearch $surfDoms $modelDoms(vtail-trailing-edge)]
set surfDoms [lreplace $surfDoms $idx $idx]

set trailingEdgeDomsCollection [pw::Collection create]
    $trailingEdgeDomsCollection set [list $modelDoms(wing-trailing-edge)  \
                                          $modelDoms(htail-trailing-edge) \
                                          $modelDoms(vtail-trailing-edge)] 

    $trailingEdgeDomsCollection do delete
$trailingEdgeDomsCollection delete

set trailingEdges [list htail-trailing-edge \
                        vtail-trailing-edge \
                        wing-trailing-edge]

foreach edge $trailingEdges {
    switch $edge {
        htail-trailing-edge {
            set conSet $horizTailTECons
        }
        vtail-trailing-edge {
            set conSet $vertTailTECons
        }
        wing-trailing-edge {
            set conSet $wingTECons
        }
    }

    set structDom [pw::DomainStructured createFromConnectors $conSet]
    set unstructDom [$structDom triangulate]
    $structDom delete
    set modelDoms($edge) $unstructDom
    lappend surfDoms $modelDoms($edge)
    $unstructDom setRenderAttribute FillMode HiddenLine
}

## Resolve Wing/Tail Leading & Trailing Edges with Anisotropic Triangles
set wingEmpennageDomsCollection [pw::Collection create]
    $wingEmpennageDomsCollection set [list $modelDoms(wing-lower)  \
                                           $modelDoms(wing-upper)  \
                                           $modelDoms(htail-lower) \
                                           $modelDoms(htail-upper) \
                                           $modelDoms(vtail)]
    $wingEmpennageDomsCollection do setUnstructuredSolverAttribute \
        TRexMaximumLayers $domLayers
    $wingEmpennageDomsCollection do setUnstructuredSolverAttribute \
        TRexGrowthRate $domGrowthRate

    set leBC [pw::TRexCondition create]

    $leBC setName "leading-edge"
    $leBC setType Wall
    $leBC setSpacing [expr {$avgDs2/$aspectRatio}]
    $leBC apply [list \
        [list $modelDoms(wing-lower) $wingLECon] \
        [list $modelDoms(wing-upper) $wingLECon] \
        [list $modelDoms(wing-lower) $wingTELowerCon] \
        [list $modelDoms(wing-upper) $wingTEUpperCon] \
        [list $modelDoms(htail-lower) $horizTailLECon] \
        [list $modelDoms(htail-upper) $horizTailLECon] \
        [list $modelDoms(htail-lower) $horizTailTELowerCon] \
        [list $modelDoms(htail-upper) $horizTailTEUpperCon] \
        [list $modelDoms(vtail) $vertTailLECon] \
        [list $modelDoms(vtail) $vertTailTEUpperCon] \
    ]

    $wingEmpennageDomsCollection do initialize
$wingEmpennageDomsCollection delete

pw::Display update

puts "Resolved Curvature at Wing/Empennage Leading & Trailing Edges"

# Refine Fuselage Surface Mesh
set symCons [difference $fuselageCons $fairingCons]
set symCons [difference $symCons $horizTailLowerCons]
set symCons [difference $symCons $horizTailUpperCons]
set symCons [difference $symCons $horizTailTECons]
set symCons [difference $symCons $vertTailCons]
set symCons [difference $symCons $vertTailTECons]

set vertTailRootNode0 [$vertTailRootCon getXYZ -parameter 0.0]
set vertTailRootNode1 [$vertTailRootCon getXYZ -parameter 1.0]

set vertTailRootTENode0 [$vertTailRootTECon getXYZ -parameter 0.0]
set vertTailRootTENode1 [$vertTailRootTECon getXYZ -parameter 1.0]

if { [lindex $vertTailRootNode0 0] < [lindex $vertTailRootNode1 0] } {
    set vertTailRootBeg [$vertTailRootCon getNode Begin]
    set vertTailRootEnd [$vertTailRootCon getNode End]
} else {
    set vertTailRootBeg [$vertTailRootCon getNode End]
    set vertTailRootEnd [$vertTailRootCon getNode Begin]
}

if { [lindex $vertTailRootTENode0 1] < [lindex $vertTailRootTENode1 1] } {
    set vertTailRootTEBeg [$vertTailRootTECon getNode Begin]
    set vertTailRootTEEnd [$vertTailRootTECon getNode End]
} else {
    set vertTailRootTEBeg [$vertTailRootTECon getNode End]
    set vertTailRootTEEnd [$vertTailRootTECon getNode Beg]
}

set conMode [pw::Application begin Modify $symCons]

    foreach con $symCons {
        $con replaceDistribution 1 [pw::DistributionGrowth create]
        set dist [$con getDistribution 1]

        set conBeg [$con getNode Begin]
        set conEnd [$con getNode End]
    
        if { [$conBeg equals $vertTailRootBeg]   || \
             [$conBeg equals $vertTailRootEnd]   || \
             [$conBeg equals $vertTailRootTEBeg] || \
             [$conBeg equals $vertTailRootTEEnd] } {
                $dist setBeginSpacing [expr {$avgDs2/$aspectRatio}]
        } elseif { [$conEnd equals $vertTailRootBeg]   || \
                   [$conEnd equals $vertTailRootEnd]   || \
                   [$conEnd equals $vertTailRootTEBeg] || \
                   [$conEnd equals $vertTailRootTEEnd] } {
                $dist setEndSpacing [expr {$avgDs2/$aspectRatio}]
        }

        $dist setBeginMode   LayersandRate
        $dist setBeginRate   $leteGrowthRate
        $dist setBeginLayers $leteLayers

        $dist setEndMode     LayersandRate
        $dist setEndRate     $leteGrowthRate
        $dist setEndLayers   $leteLayers
        
        $con setDimensionFromDistribution
    }
$conMode end

puts "Surface Meshing Complete"

pw::Display update

# Create Farfield
set modelExtent [pw::Grid getExtents]
set minExtent [lindex $modelExtent 0]
set maxExtent [lindex $modelExtent 1]
set center [pwu::Vector3 divide [pwu::Vector3 add $minExtent $maxExtent] 2.0]
set center "[pwu::Vector3 x $center] 0.0 [pwu::Vector3 z $center]"

set ff [UnsHemisphere [expr {$ffSize*$cLen}] $center $avgDs3]

set ffDoms [lindex $ff 0]
set ffSymCons [lindex $ff 1]

# Create Symmetry Plane

## Isolate Connectors From Analysis Model Symmetry Edge
set modelFace [pw::FaceUnstructured createFromDomains $surfDoms]
set edge [$modelFace getEdge 1]

for { set i 1 } { $i <= [$edge getConnectorCount] } { incr i } {
    lappend consInSym [$edge getConnector $i]
}

set symMode [pw::Application begin Create]
    set symDom [pw::DomainUnstructured create]
        set extEdge [pw::Edge createFromConnectors $ffSymCons]
        $symDom addEdge $extEdge
        set intEdge [pw::Edge createFromConnectors $consInSym]
        $symDom addEdge $intEdge

        # Older versions of Pointwise could generate empty domains
        # when an interior edge is reversed. Newer versions
        # may still do this, but may also produce folded and
        # intersecting cells. In either case, reverse the
        # interior edge.
        if { [$symDom getCellCount] == 0 } {
            # Empty domain
            $intEdge reverse
        } else {
            # Check for cell intersection and reverse the interior edge,
            # if necessary
            set examiner [pw::Examine create DomainCellIntersection]
            $examiner addEntity $symDom
            $examiner examine
            if { [$examiner getCategoryCount Intersected] != 0 } {
                $intEdge reverse
            }
            $examiner delete
        }

$symMode end
unset symMode

## Update Display Color for Symmetry Plane
$symDom setRenderAttribute ColorMode Entity
$symDom setColor "#FFFF00"

pw::Display update

# Create Farfield Block

## Retrieve Outer Farfield, Symmetry, and Analysis Model Domains
set ffBlkDoms [join [list $ffDoms $symDom $surfDoms]]

## Assemble & Initialize Block
set ffBlk [pw::BlockUnstructured createFromDomains $ffBlkDoms]
$ffBlk setName "aniso-blk"

## Initialize T-Rex Block
set trexMode [pw::Application begin UnstructuredSolver $ffBlk]
    $ffBlk setUnstructuredSolverAttribute TRexMaximumLayers $numLayers
    $ffBlk setUnstructuredSolverAttribute TRexGrowthRate $growthRate
    $ffBlk setUnstructuredSolverAttribute TRexFullLayers $fullLayers
    $ffBlk setUnstructuredSolverAttribute TRexPushAttributes True
    $ffBlk setUnstructuredSolverAttribute TRexCollisionBuffer $collisionBuffer
    $ffBlk setUnstructuredSolverAttribute TRexSkewCriteriaMaximumAngle $maxAngle
    $ffBlk setUnstructuredSolverAttribute TRexSkewCriteriaCentroid $centroidSkew
    $ffBlk setUnstructuredSolverAttribute BoundaryDecay $boundaryDecay

    set modelBC [pw::TRexCondition create]
        $modelBC setName    "aircraft"
        $modelBC setType    "Wall"
        $modelBC setSpacing $initDs

        foreach dom $surfDoms {
            $modelBC apply [list [list $ffBlk $dom]]
        }

    set symBC [pw::TRexCondition create]
        $symBC setName "symmetry"
        $symBC setType "Match"
        $symBC apply [list [list $ffBlk $symDom]]

$trexMode run Initialize
$trexMode end

pw::Display update

puts "Anisotropic tetrahedra created"

# Examine Blocks

## Number of Cells
puts "[$ffBlk getCellCount] tetrahedra"
puts "[$ffBlk getTRexFullLayerCount] full layers"
puts "[$ffBlk getTRexTotalLayerCount] total layers"

## Maximum Included Angle
set blkExm [pw::Examine create BlockMaximumAngle]
    $blkExm addEntity $ffBlk
    $blkExm setRangeLimits 70.0 175.0
    $blkExm examine

    puts "[format "Maximum included angle: %.3f" [$blkExm getMaximum]]"
    puts "Number of cells above maximum (175.0): [$blkExm getAboveRange]"
$blkExm delete

## Aspect Ratio
set blkExm [pw::Examine create BlockAspectRatio]
    $blkExm addEntity $ffBlk
    $blkExm examine
    puts "[format "Maximum aspect ratio: %.3f" [$blkExm getMaximum]]"
$blkExm delete

## Centroid Skew
set blkExm [pw::Examine create BlockSkewCentroid]
    $blkExm addEntity $ffBlk
    $blkExm setRangeLimits 0.4 0.9
    $blkExm examine

    puts "[format "Maximum centroid skew angle: %.3f" [$blkExm getMaximum]]"
    puts "Number of cells above maximum (0.9): [$blkExm getAboveRange]"
$blkExm delete

pw::Display update

# CAE Setup
pw::Application setCAESolver "CGNS"

# Define boundary conditions
set wallBC [pw::BoundaryCondition create]
    $wallBC setName "aircraft"
    $wallBC setPhysicalType "Wall"
    foreach dom $surfDoms {$wallBC apply [list $ffBlk $dom]}

set symBC [pw::BoundaryCondition create]
    $symBC setName "symmetry"
    $symBC setPhysicalType "Symmetry Plane"
    $symBC apply [list $ffBlk $symDom]

set ffBC [pw::BoundaryCondition create]
    $ffBC setName "farfield"
    $ffBC setPhysicalType "Farfield"
    foreach dom $ffDoms {$ffBC apply [list $ffBlk $dom]}

# Define volume condition
set fluidVC [pw::VolumeCondition create]
    $fluidVC setName "fluid"
    $fluidVC setPhysicalType "Fluid"
    $fluidVC apply $ffBlk

timestamp
puts "Run Time: [convSeconds [pwu::Time elapsed $tBegin]]"

# Save the Pointwise Project
set fileRoot [file rootname $fileName]
set fileExport "$fileRoot-Grid.pw"

puts ""
puts "Writing $fileExport file..."
puts ""

pw::Application save [file join $scriptDir $fileExport]

pw::Display update
exit

#############################################################################
#
# This file is licensed under the Cadence Public License Version 1.0 (the
# "License"), a copy of which is found in the included file named "LICENSE",
# and is distributed "AS IS." TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE
# LAW, CADENCE DISCLAIMS ALL WARRANTIES AND IN NO EVENT SHALL BE LIABLE TO
# ANY PARTY FOR ANY DAMAGES ARISING OUT OF OR RELATING TO USE OF THIS FILE.
# Please see the License for the full text of applicable terms.
#
#############################################################################
