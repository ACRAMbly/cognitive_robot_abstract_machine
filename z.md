a synthetic annotator, with the same scenarios exercised through pytest. The proposed structure is:

  ┌───────────────────────┐  ┌───────────────────────┐  ┌──────────────────────────────┐  ┌───────────────────────┐  ┌───────────────────┐  ┌──────────────────────────┐
  │ Scripted observations │  │ CAS object hypotheses │  │ SemanticDigitalTwinConnector │  │ RoboKudo belief world │  │ WorldSynchronizer │  │ Separate receiving world │
  └┬──────────────────────┘  └┬─┬────────────────────┘  └┬─┬───────────────────────────┘  └┬─┬────────────────────┘  └┬─┬────────────────┘  └┬─────────────────────────┘
   │                          ▲ │                        ▲ │                               ▲ │                        ▲ │                    ▲
   │                          │ │                        │ │                               │ │                        │ │                    │
   │                          │ │                        │ │                               │ │                        │ │                    │
   │                          │ │                        │ │                               │ │                        │ │                    │
   └──────────────────────────┘ │                        │ │                               │ │                        │ │                    │
                                │                        │ │                               │ │                        │ │                    │
                                └────────────────────────┘ │                               │ │                        │ │                    │
                                                           │                               │ │                        │ │                    │
                                                           └───────────────────────────────┘ │                        │ │                    │
                                                                                             │                        │ │                    │
                                                                                             └────────────────────────┘ │                    │
                                                                                                                        │                    │
                                                                                                                        └────────────────────┘

  This is a minimal test analysis engine: an observation source followed by the existing connector.

  ───────────────────────────────────────────────────────────────────────────────────  ───────────────────────────────────────────────  ───────────────────────────────
   Preloaded CAD mesh through ObjectSpec (robokudo/src/robokudo/                         Mesh plus enclosing box                          Enclosing box
   world_descriptor.py:121)

   sythetic data for our specific two tettris blocks.