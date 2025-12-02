#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/errorgraph/errorgraph.h>

using namespace mrs_lib::errorgraph;

class ErrorgraphTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    graph_ = std::make_unique<Errorgraph>(clock_);
  }

  // Helper to create a simple errorgraph element message
  errorgraph_element_msg_t create_element_msg(const std::string& node, const std::string& component,
                                              const std::vector<std::pair<std::string, std::string>>& error_types = {})
  {
    errorgraph_element_msg_t msg;
    msg.stamp = clock_->now();
    msg.source_node.node = node;
    msg.source_node.component = component;

    for (const auto& [error_type, waited_for] : error_types)
    {
      errorgraph_error_msg_t error_msg;
      error_msg.stamp = clock_->now();
      error_msg.type = error_type;

      if (error_type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE && !waited_for.empty())
      {
        // Parse waited_for as "node.component"
        size_t pos = waited_for.find('.');
        if (pos != std::string::npos)
        {
          error_msg.waited_for_node.node = waited_for.substr(0, pos);
          error_msg.waited_for_node.component = waited_for.substr(pos + 1);
        }
      }
      else if (error_type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC && !waited_for.empty())
      {
        error_msg.waited_for_topic = waited_for;
      }

      msg.errors.push_back(error_msg);
    }

    return msg;
  }

  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<Errorgraph> graph_;
};

/* TEST: find_error_roots with empty graph //{ */

TEST_F(ErrorgraphTest, find_error_roots_empty_graph)
{
  auto roots = graph_->find_error_roots();
  EXPECT_TRUE(roots.empty());
}

//}

/* TEST: find_error_roots with single node no error //{ */

TEST_F(ErrorgraphTest, find_error_roots_single_node_no_error)
{
  // Add a node with no errors
  auto msg = create_element_msg("test_node", "test_component", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});
  graph_->add_element_from_msg(msg);

  auto roots = graph_->find_error_roots();
  EXPECT_TRUE(roots.empty());
}

//}

/* TEST: find_error_roots with single node having error //{ */

TEST_F(ErrorgraphTest, find_error_roots_single_node_with_error)
{
  // Add a node with an actual error
  auto msg = create_element_msg("test_node", "test_component", {{"CUSTOM_ERROR", ""}});
  graph_->add_element_from_msg(msg);

  auto roots = graph_->find_error_roots();
  ASSERT_EQ(roots.size(), 1);
  EXPECT_EQ(roots[0]->source_node.node, "test_node");
  EXPECT_EQ(roots[0]->source_node.component, "test_component");
}

//}

/* TEST: find_error_roots with multiple error roots //{ */

TEST_F(ErrorgraphTest, find_error_roots_multiple_error_roots)
{
  // Add multiple nodes with errors (none waiting for others)
  auto msg1 = create_element_msg("node1", "component1", {{"ERROR_TYPE_A", ""}});
  auto msg2 = create_element_msg("node2", "component2", {{"ERROR_TYPE_B", ""}});
  auto msg3 = create_element_msg("node3", "component3", {{"ERROR_TYPE_C", ""}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);
  graph_->add_element_from_msg(msg3);

  auto roots = graph_->find_error_roots();
  EXPECT_EQ(roots.size(), 3);
}

//}

/* TEST: find_error_roots with node waiting for another node //{ */

TEST_F(ErrorgraphTest, find_error_roots_node_waiting_for_dependency)
{
  // Node2 is waiting for Node1
  auto msg1 = create_element_msg("node1", "component1", {{"ERROR_TYPE_A", ""}});
  auto msg2 = create_element_msg("node2", "component2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node1.component1"}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);

  auto roots = graph_->find_error_roots();

  // Node2 should NOT be a root because it's waiting for node1
  // Node1 should be a root because it has an error and isn't waiting for anything
  ASSERT_EQ(roots.size(), 1);
  EXPECT_EQ(roots[0]->source_node.node, "node1");
  EXPECT_EQ(roots[0]->source_node.component, "component1");
}

//}

/* TEST: find_error_roots excludes no-error nodes //{ */

TEST_F(ErrorgraphTest, find_error_roots_excludes_no_error_nodes)
{
  // Mix of nodes: some with errors, some without
  auto msg1 = create_element_msg("node_error", "component1", {{"ERROR_TYPE_A", ""}});
  auto msg2 = create_element_msg("node_ok", "component2", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});
  auto msg3 = create_element_msg("node_error2", "component3", {{"ERROR_TYPE_B", ""}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);
  graph_->add_element_from_msg(msg3);

  auto roots = graph_->find_error_roots();

  // Only nodes with errors should be roots (node_ok should be excluded)
  EXPECT_EQ(roots.size(), 2);

  bool found_node_error = false;
  bool found_node_error2 = false;
  bool found_node_ok = false;

  for (const auto* root : roots)
  {
    if (root->source_node.node == "node_error")
      found_node_error = true;
    if (root->source_node.node == "node_ok")
      found_node_ok = true;
    if (root->source_node.node == "node_error2")
      found_node_error2 = true;
  }

  EXPECT_TRUE(found_node_error);
  EXPECT_TRUE(found_node_error2);
  EXPECT_FALSE(found_node_ok);
}

//}

/* TEST: find_error_roots chain of dependencies //{ */

TEST_F(ErrorgraphTest, find_error_roots_chain_of_dependencies)
{
  // Chain: node3 -> node2 -> node1 (where -> means "waiting for")
  // node1 has error, node2 waits for node1, node3 waits for node2
  auto msg1 = create_element_msg("node1", "comp1", {{"ROOT_ERROR", ""}});
  auto msg2 = create_element_msg("node2", "comp2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node1.comp1"}});
  auto msg3 = create_element_msg("node3", "comp3", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node2.comp2"}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);
  graph_->add_element_from_msg(msg3);

  auto roots = graph_->find_error_roots();

  // Only node1 should be an error root because it has error and isn't waiting
  ASSERT_EQ(roots.size(), 1);
  EXPECT_EQ(roots[0]->source_node.node, "node1");
}

//}

/* TEST: find_error_roots with waiting node and no-error at dependency root //{ */

TEST_F(ErrorgraphTest, find_error_roots_waiting_with_no_error_dependency)
{
  // node2 is waiting for node1, but node1 has no errors
  auto msg1 = create_element_msg("node1", "comp1", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});
  auto msg2 = create_element_msg("node2", "comp2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node1.comp1"}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);

  auto roots = graph_->find_error_roots();

  // node1 has no error so it's not a root, node2 is waiting so it's not a root either
  EXPECT_TRUE(roots.empty());
}

//}

/* TEST: find_error_roots with multiple errors on same node //{ */

TEST_F(ErrorgraphTest, find_error_roots_multiple_errors_same_node)
{
  // Node with both a waiting-for error and a regular error
  errorgraph_element_msg_t msg;
  msg.stamp = clock_->now();
  msg.source_node.node = "multi_error_node";
  msg.source_node.component = "component";

  // Add waiting-for error
  errorgraph_error_msg_t wait_error;
  wait_error.stamp = clock_->now();
  wait_error.type = errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE;
  wait_error.waited_for_node.node = "other_node";
  wait_error.waited_for_node.component = "other_comp";
  msg.errors.push_back(wait_error);

  // Also add regular error
  errorgraph_error_msg_t regular_error;
  regular_error.stamp = clock_->now();
  regular_error.type = "REGULAR_ERROR";
  msg.errors.push_back(regular_error);

  graph_->add_element_from_msg(msg);

  auto roots = graph_->find_error_roots();

  // Node should NOT be a root because it's waiting (even though it also has a regular error)
  EXPECT_TRUE(roots.empty());
}

//}

/* TEST: find_error_roots with node waiting for topic //{ */

TEST_F(ErrorgraphTest, find_error_roots_waiting_for_topic)
{
  // Node is waiting for a topic
  auto msg = create_element_msg("waiting_node", "component", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC, "/some/topic"}});
  graph_->add_element_from_msg(msg);

  auto roots = graph_->find_error_roots();

  // Node waiting for topic should NOT be an error root
  EXPECT_TRUE(roots.empty());
}

//}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
