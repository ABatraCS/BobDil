within BobDil;

package Experiments "BobDil-specific Modelica entry points"
  annotation(Documentation(info = "<html><p>
Modelica that belongs to BobDil rather than to BobLib. BobLib is consumed
read-only; anything BobDil needs that BobLib does not provide lives here, so
that BobDil can be built and run without patching its dependency.
</p></html>"));
end Experiments;
