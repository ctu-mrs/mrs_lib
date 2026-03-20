#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/errorgraph/errorgraph.h>

using namespace mrs_lib::errorgraph;

class ErrorgraphTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
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
      } else if (error_type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC && !waited_for.empty())
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
  const auto& info = std::get<Errorgraph::node_info_t>(roots[0]);
  EXPECT_EQ(info.source_node.node, "test_node");
  EXPECT_EQ(info.source_node.component, "test_component");
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
  const auto& info = std::get<Errorgraph::node_info_t>(roots[0]);
  EXPECT_EQ(info.source_node.node, "node1");
  EXPECT_EQ(info.source_node.component, "component1");
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

  for (const auto& root : roots)
  {
    const auto& info = std::get<Errorgraph::node_info_t>(root);
    if (info.source_node.node == "node_error")
      found_node_error = true;
    if (info.source_node.node == "node_ok")
      found_node_ok = true;
    if (info.source_node.node == "node_error2")
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
  const auto& info = std::get<Errorgraph::node_info_t>(roots[0]);
  EXPECT_EQ(info.source_node.node, "node1");
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
  errorgraph_element_msg_t msg = create_element_msg("multi_error_node", "component");

  errorgraph_error_msg_t wait_error;
  wait_error.stamp = clock_->now();
  wait_error.type = errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE;
  wait_error.waited_for_node.node = "other_node";
  wait_error.waited_for_node.component = "other_comp";
  msg.errors.push_back(wait_error);

  errorgraph_error_msg_t regular_error;
  regular_error.stamp = clock_->now();
  regular_error.type = "REGULAR_ERROR";
  msg.errors.push_back(regular_error);

  graph_->add_element_from_msg(msg);

  auto roots = graph_->find_error_roots();

  // "other_node" becomes a root because it doesn't exist (not reporting)
  // "multi_error_node" is NOT a root because it's waiting
  ASSERT_EQ(roots.size(), 1);
  const auto& info = std::get<Errorgraph::node_info_t>(roots[0]);
  EXPECT_EQ(info.source_node.node, "other_node");
  EXPECT_EQ(info.source_node.component, "other_comp");
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

/* TEST: find_dependency_roots //{ */

TEST_F(ErrorgraphTest, find_dependency_roots_simple_chain)
{
  // Create chain: node3 -> node2 -> node1
  auto msg1 = create_element_msg("node1", "comp1", {{"ERROR", ""}});
  auto msg2 = create_element_msg("node2", "comp2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node1.comp1"}});
  auto msg3 = create_element_msg("node3", "comp3", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node2.comp2"}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);
  graph_->add_element_from_msg(msg3);

  node_id_t node3_id{"node3", "comp3"};
  auto roots = graph_->find_dependency_roots(node3_id);

  // Should trace back to node1
  ASSERT_EQ(roots.size(), 1);
  const auto& info = std::get<Errorgraph::node_info_t>(roots[0]);
  EXPECT_EQ(info.source_node.node, "node1");
}

//}

/* TEST: find_dependency_roots detects loops //{ */

TEST_F(ErrorgraphTest, find_dependency_roots_detects_loop)
{
  // Create circular dependency: node1 -> node2 -> node1
  auto msg1 = create_element_msg("node1", "comp1", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node2.comp2"}});
  auto msg2 = create_element_msg("node2", "comp2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node1.comp1"}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);

  node_id_t node1_id{"node1", "comp1"};
  bool loop_detected = false;
  auto roots = graph_->find_dependency_roots(node1_id, &loop_detected);

  EXPECT_TRUE(loop_detected);
  EXPECT_FALSE(roots.empty());
}

//}

/* TEST: find_roots returns all non-waiting nodes //{ */

TEST_F(ErrorgraphTest, find_roots_returns_all_non_waiting_nodes)
{
  // Create mix: some waiting, some not
  auto msg1 = create_element_msg("root1", "comp1", {{"ERROR", ""}});
  auto msg2 = create_element_msg("waiting", "comp2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "root1.comp1"}});
  auto msg3 = create_element_msg("root2", "comp3", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);
  graph_->add_element_from_msg(msg3);

  auto roots = graph_->find_roots();

  // Should return both root1 and root2 (both not waiting)
  EXPECT_EQ(roots.size(), 2);
}

//}

/* TEST: find_leaves identifies leaf nodes //{ */

TEST_F(ErrorgraphTest, find_leaves_identifies_leaf_nodes)
{
  // node1 <- node2 <- node3 (arrows show "waiting for" direction)
  // node1 is a leaf (nobody waits for it)
  auto msg1 = create_element_msg("node3", "comp3", {{"ERROR", ""}});
  auto msg2 = create_element_msg("node2", "comp2", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node3.comp3"}});
  auto msg3 = create_element_msg("node1", "comp1", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE, "node2.comp2"}});
  // node1 is waiting for node2, which is waiting for node3

  graph_->add_element_from_msg(msg1);
  graph_->add_element_from_msg(msg2);
  graph_->add_element_from_msg(msg3);

  auto leaves = graph_->find_leaves();

  // node1 is a leaf (nobody depends on it)
  ASSERT_EQ(leaves.size(), 1);
  const auto& leaf_info = std::get<Errorgraph::node_info_t>(leaves[0]);
  EXPECT_EQ(leaf_info.source_node.node, "node1");

  // Find roots should return node3
  // node3 is a root (nobody it waits for has error)
  auto roots = graph_->find_roots();
  ASSERT_EQ(roots.size(), 1);
  const auto& root_info = std::get<Errorgraph::node_info_t>(roots[0]);
  EXPECT_EQ(root_info.source_node.node, "node3");
}

//}

/* TEST: topic element creation when waiting for topic //{ */

TEST_F(ErrorgraphTest, waiting_for_topic_creates_topic_element)
{
  auto msg = create_element_msg("node1", "comp1", {{errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC, "/test/topic"}});
  graph_->add_element_from_msg(msg);


  // Trigger graph building (prepare_graph creates missing elements)
  graph_->find_error_roots();

  // Check that topic element was created
  auto topic_elem = graph_->find_element("/test/topic");
  ASSERT_TRUE(topic_elem.has_value());
  ASSERT_TRUE(std::holds_alternative<Errorgraph::topic_info_t>(topic_elem.value()));
  const auto& topic = std::get<Errorgraph::topic_info_t>(topic_elem.value());
  EXPECT_EQ(topic.topic_name, "/test/topic");
}

//}

/* TEST: node becomes not reporting after delay //{ */
TEST_F(ErrorgraphTest, node_becomes_not_reporting_after_delay)
{
  auto msg = create_element_msg("old_node", "comp", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});
  // Set timestamp far in the past
  msg.stamp = rclcpp::Time(static_cast<uint64_t>(0), RCL_ROS_TIME);
  graph_->add_element_from_msg(msg);
  // Trigger graph building
  auto elem = graph_->find_element(node_id_t{"old_node", "comp"});
  ASSERT_TRUE(elem.has_value());
  const auto& info = std::get<Errorgraph::node_info_t>(elem.value());
  EXPECT_TRUE(info.not_reporting);

  // Now add a recent message
  auto recent_msg = create_element_msg("old_node", "comp", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});
  graph_->add_element_from_msg(recent_msg);
  auto elem2 = graph_->find_element(node_id_t{"old_node", "comp"});
  ASSERT_TRUE(elem2.has_value());
  const auto& info2 = std::get<Errorgraph::node_info_t>(elem2.value());
  EXPECT_FALSE(info2.not_reporting);
}

//}

/* TEST: graph rebuilds after updates //{ */

TEST_F(ErrorgraphTest, graph_rebuilds_after_updates)
{
  auto msg1 = create_element_msg("node1", "comp1", {{"ERROR", ""}});
  graph_->add_element_from_msg(msg1);

  auto roots1 = graph_->find_error_roots();
  EXPECT_EQ(roots1.size(), 1);

  // Update node to no error
  auto msg2 = create_element_msg("node1", "comp1", {{errorgraph_error_msg_t::TYPE_NO_ERROR, ""}});
  graph_->add_element_from_msg(msg2);

  auto roots2 = graph_->find_error_roots();
  EXPECT_TRUE(roots2.empty());
}

//}

/* TEST: find_element returns nullopt for nonexistent elements //{ */

TEST_F(ErrorgraphTest, find_element_returns_nullopt_for_nonexistent)
{
  auto elem = graph_->find_element(node_id_t{"nonexistent", "node"});
  EXPECT_FALSE(elem.has_value());

  auto topic = graph_->find_element("/nonexistent/topic");
  EXPECT_FALSE(topic.has_value());
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
