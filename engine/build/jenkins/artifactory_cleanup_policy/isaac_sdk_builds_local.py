def purgelist(artifactory):
  compute_jetson_terms = [{
      "$and": [
          {"updated": {"$before": "7d"}},
          {"path": {"$match": "jetson_tests"}}
      ]
  }]
  jetson_purgable = artifactory.filter(terms=compute_jetson_terms, depth=None, item_type="any")

  compute_x86_terms = [{
      "$and": [
          {"updated": {"$before": "7d"}},
          {"path": {"$match": "x86_tests"}}
      ]
  }]
  x86_purgable = artifactory.filter(terms=compute_x86_terms, depth=None, item_type="any")

  compute_nightly_master_terms = [{
      "$and": [
          {"updated": {"$before": "30d"}},
          {"path": {"$match": "nightly/master"}}
      ]
  }]
  nightly_master_purgable = artifactory.filter(terms=compute_nightly_master_terms, depth=None, item_type="any")

  compute_nightly_test_terms = [{
      "$and": [
          {"updated": {"$before": "7d"}},
          {"path": {"$match": "nightly/pipeline-testing"}}
      ]
  }]
  nightly_test_purgable = artifactory.filter(terms=compute_nightly_test_terms, depth=None, item_type="any")

  compute_nightly_deploy_terms = [{
      "$and": [
          {"updated": {"$before": "3d"}},
          {"path": {"$match": "deployment_nightly"}}
      ]
  }]
  nightly_deploy_purgable = artifactory.filter(terms=compute_nightly_deploy_terms, depth=None, item_type="any")

  purgable = jetson_purgable + x86_purgable + nightly_master_purgable + nightly_test_purgable + nightly_deploy_purgable

  return purgable